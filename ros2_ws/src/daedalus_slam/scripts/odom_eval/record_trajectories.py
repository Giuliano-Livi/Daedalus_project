#!/usr/bin/env python3
"""Records ground truth and every odometry estimate of a flight as TUM files, for evaluate.py.

Run inside the dev container next to the flight (run_eval.sh) or a bag replay (run_replay.sh):
    python3 record_trajectories.py --out /ws/images/odom_eval/<run> [--gt-source ros] [--mode ...]

TUM format, one pose per line: t x y z qx qy qz qw (t = sim time, s).
  gt.tum          Gazebo model pose (world frame), read from gz-transport (the ROS bridge of
                  Pose_V drops the model names and stamps) and republished on /gt/odom, so that a
                  bag of the flight contains it; --gt-source ros reads /gt/odom (bag replay)
  vo.tum          rgbd_odometry (/vo/odom, frame vo_odom); lost frames go to vo_lost.txt
  ekf.tum         PX4 EKF2 (/odom from px4_odometry, frame odom)
  slam.tum        online SLAM estimate, TF map -> base_link sampled at 20 Hz
(slam_final.tum, the optimized graph the map is built on, is exported from rtabmap.db by run_eval.sh:
/rtabmap/mapPath has no per-node stamps)

It stops when the trajectory is over (setpoint unchanged for --end-hold s), on a crash (drone back
on the ground after takeoff), when sim time stops (end of a bag replay) or after --max-time s of sim time.
"""

import argparse
import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from daedalus_interfaces.msg import FlightSetpoint
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

try:
    from rtabmap_msgs.msg import OdomInfo
except ImportError:
    OdomInfo = None

LOST_COV = 9000.0    # rgbd_odometry publishes a null pose with covariance 9999 when lost
# crash: after takeoff, tilt above CRASH_TILT, or below CRASH_HEIGHT for CRASH_TIME (it can end on a pallet,
# not on the floor)
CRASH_TILT = 70.0    # deg
CRASH_HEIGHT = 0.5   # m above the start
CRASH_TIME = 2.0     # s


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, args):
        super().__init__('odom_eval_recorder',
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.args = args
        self.lock = threading.Lock()
        self.traj = {k: [] for k in ('gt', 'vo', 'ekf', 'slam')}
        self.vo_lost = []
        self.odom_info = []          # t, lost, inliers, features
        self.done_reason = None

        # flight state (ground truth)
        self.z0 = None
        self.airborne = False
        self.t_takeoff = None
        self.last_sp = None
        self.t_sp_change = None
        self.sp_moved = False
        self.t_crash = None
        self.t_low = None

        self.last_sim, self.last_sim_wall = None, time.monotonic()
        if args.gt_source == 'gz':
            self.gt_pub = self.create_publisher(Odometry, '/gt/odom', 50)
            self.gt_published = 0
            self.create_timer(0.02, self.publish_gt, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.gz = GzNode()
            topic = f'/world/{args.world}/dynamic_pose/info'
            if not self.gz.subscribe(Pose_V, topic, self.on_gz_poses):
                raise RuntimeError(f'cannot subscribe to {topic}')
        else:
            self.create_subscription(Odometry, '/gt/odom', self.on_gt_odom, 100)

        self.create_subscription(Odometry, '/vo/odom', self.on_vo, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_ekf, 50)
        self.create_subscription(FlightSetpoint, '/drone/setpoint', self.on_setpoint, 10)
        if OdomInfo is not None:
            self.create_subscription(OdomInfo, '/odom_info', self.on_odom_info, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_slam_t = None
        self.create_timer(0.05, self.sample_slam)
        # wall-clock timer: it has to run also when sim time stops (end of a replay)
        self.create_timer(1.0, self.check_end, clock=Clock(clock_type=ClockType.STEADY_TIME))

    # ground truth, called from the gz-transport thread
    def on_gz_poses(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name != self.args.model:
                continue
            row = (t, p.position.x, p.position.y, p.position.z,
                   p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            # published on /gt/odom by the main thread (publish_gt): calling rclpy from the gz-transport
            # thread blocked the main loop for seconds (GIL) and crashed at shutdown
            self.add_gt(row)
            return

    def publish_gt(self):
        with self.lock:
            rows = self.traj['gt'][self.gt_published:]
            self.gt_published = len(self.traj['gt'])
        for row in rows:
            out = Odometry()
            ns = int(round(row[0] * 1e9))
            out.header.stamp.sec, out.header.stamp.nanosec = divmod(ns, 1000000000)
            out.header.frame_id, out.child_frame_id = 'world', 'gt_base_link'
            pp, q = out.pose.pose.position, out.pose.pose.orientation
            pp.x, pp.y, pp.z, q.x, q.y, q.z, q.w = row[1:]
            self.gt_pub.publish(out)

    def on_gt_odom(self, msg):
        self.add_gt(self.odom_row(msg))

    def add_gt(self, row):
        t, z = row[0], row[3]
        qx, qy = row[4], row[5]
        tilt = np.degrees(2 * np.arcsin(min(1.0, np.hypot(qx, qy))))   # angle of body z from vertical
        with self.lock:
            if self.traj['gt'] and t <= self.traj['gt'][-1][0]:
                return False
            self.traj['gt'].append(row)
            if self.z0 is None:
                self.z0 = z
            h = z - self.z0
            if not self.airborne and h > 1.0:
                self.airborne, self.t_takeoff = True, t
            if self.airborne and self.t_crash is None:
                low = h < CRASH_HEIGHT
                self.t_low = (self.t_low if self.t_low is not None else t) if low else None
                if tilt > CRASH_TILT or (low and t - self.t_low >= CRASH_TIME):
                    self.t_crash = t
        return True

    @staticmethod
    def odom_row(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return (stamp_s(msg.header.stamp), p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def on_vo(self, msg):
        row = self.odom_row(msg)
        with self.lock:
            if msg.pose.covariance[0] >= LOST_COV or row[4:] == (0.0, 0.0, 0.0, 0.0):
                self.vo_lost.append(row[0])
            else:
                self.traj['vo'].append(row)

    def on_ekf(self, msg):
        with self.lock:
            self.traj['ekf'].append(self.odom_row(msg))

    def on_odom_info(self, msg):
        with self.lock:
            self.odom_info.append((stamp_s(msg.header.stamp), int(msg.lost), msg.inliers, msg.features))

    def sample_slam(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except Exception:
            return
        t = stamp_s(tf.header.stamp)
        if self.last_slam_t is not None and t <= self.last_slam_t:
            return
        self.last_slam_t = t
        tr, q = tf.transform.translation, tf.transform.rotation
        with self.lock:
            self.traj['slam'].append((t, tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w))

    def on_setpoint(self, msg):
        sp = (round(msg.position.x, 3), round(msg.position.y, 3), round(msg.position.z, 3), round(msg.yaw, 3))
        now = self.get_clock().now().nanoseconds * 1e-9
        if sp != self.last_sp:
            if self.last_sp is not None:
                self.sp_moved = True
            self.last_sp, self.t_sp_change = sp, now

    def check_end(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now != self.last_sim:
            self.last_sim, self.last_sim_wall = now, time.monotonic()
        with self.lock:
            n = {k: len(v) for k, v in self.traj.items()}
            t_crash = self.t_crash
        self.get_logger().info(
            f't={now:.0f}s gt={n["gt"]} vo={n["vo"]} (lost {len(self.vo_lost)}) ekf={n["ekf"]} '
            f'slam={n["slam"]}', throttle_duration_sec=10.0)
        if t_crash is not None and now - t_crash > 3.0:
            self.done_reason = f'crash at t={t_crash:.1f}s'
        elif self.sp_moved and self.t_sp_change is not None and now - self.t_sp_change > self.args.end_hold:
            self.done_reason = 'trajectory completed'
        elif self.args.gt_source == 'ros' and n['gt'] and time.monotonic() - self.last_sim_wall > self.args.stall:
            # only in a replay: in a live flight a pause of the clock is not the end
            self.done_reason = f'sim time stopped for {self.args.stall:g} s (end of data)'
        elif self.t_takeoff is not None and now - self.t_takeoff > self.args.max_time:
            self.done_reason = 'max time'

    def save(self):
        os.makedirs(self.args.out, exist_ok=True)
        with self.lock:
            for name, rows in self.traj.items():
                with open(os.path.join(self.args.out, f'{name}.tum'), 'w') as f:
                    for r in sorted(rows):
                        f.write(' '.join(f'{v:.9f}' if i == 0 else f'{v:.6f}' for i, v in enumerate(r)) + '\n')
            with open(os.path.join(self.args.out, 'vo_lost.txt'), 'w') as f:
                f.writelines(f'{t:.9f}\n' for t in self.vo_lost)
            with open(os.path.join(self.args.out, 'odom_info.txt'), 'w') as f:
                f.write('# t lost inliers features\n')
                f.writelines(f'{t:.9f} {l} {i} {ft}\n' for t, l, i, ft in self.odom_info)
            meta = {'mode': self.args.mode, 'end': self.done_reason, 't_takeoff': self.t_takeoff, 't_crash': self.t_crash,
                    'counts': {k: len(v) for k, v in self.traj.items()}}
        with open(os.path.join(self.args.out, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(f'saved to {self.args.out}: {meta}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--world', default='warehouse_px4')
    ap.add_argument('--model', default='x500_depth_0')
    ap.add_argument('--end-hold', type=float, default=8.0, help='s of unchanged setpoint = trajectory over')
    ap.add_argument('--max-time', type=float, default=1200.0, help='s of sim time after takeoff')
    ap.add_argument('--stall', type=float, default=5.0, help='wall s without sim time progress = end of data')
    ap.add_argument('--gt-source', choices=('gz', 'ros'), default='gz')
    ap.add_argument('--mode', default='closed_loop',
                    help='closed_loop (no GPS, flies on vision), shadow (flies on GPS, vision not used) or replay')
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args)
    try:
        while rclpy.ok() and node.done_reason is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.done_reason = 'interrupted'
    node.get_logger().info(f'end: {node.done_reason}')
    node.save()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
