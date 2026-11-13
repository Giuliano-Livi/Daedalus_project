#!/usr/bin/env python3
"""Accuracy of the floor height (/floor/height, daedalus_slam/manhattan_yaw) against the ground truth
(/gt/odom: base_link height minus the floor level --floor-z), on a bag replay (odom_replay.launch.py):

    ros2 launch daedalus_slam odom_replay.launch.py estimator:=openvins &
    python3 eval_floor.py --out /ws/images/odom_eval/<run> &
    ros2 bag play /ws/images/odom_bags/<bag>

Writes floor.md (error, bias and availability in flight, per minimum number of points) and floor.png.
"""
import argparse, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from daedalus_interfaces.msg import FloorHeight
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

MIN_POINTS = (0, 50, 100, 200, 400)


class Rec(Node):
    def __init__(self):
        super().__init__('eval_floor', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.f, self.g = [], []
        self.create_subscription(FloorHeight, '/floor/height', lambda m: self.f.append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.height, m.num_points, m.spread)), 50)
        self.create_subscription(Odometry, '/gt/odom', lambda m: self.g.append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.z)), 200)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--floor-z', type=float, default=0.002, help='world z of the floor surface (warehouse_floor)')
    ap.add_argument('--stall', type=float, default=5.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    n = Rec()
    last, wall = 0, time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if len(n.g) != last:
            last, wall = len(n.g), time.monotonic()
        elif last and time.monotonic() - wall > args.stall:
            break
    f, g = np.array(n.f), np.array(sorted(n.g))
    rclpy.try_shutdown()
    h_gt = np.interp(f[:, 0], g[:, 0], g[:, 1]) - args.floor_z
    err = f[:, 1] - h_gt
    fly = h_gt > 1.0
    t_fly = g[:, 0][g[:, 1] - args.floor_z > 1.0]
    n_fly_frames = len(f[fly]) if fly.any() else 0
    dur = t_fly[-1] - t_fly[0] if len(t_fly) else 1.0
    L = ['# Floor height accuracy', '', f'{len(f)} measurements, {fly.sum()} in flight ({fly.sum() / dur:.1f} Hz over '
         f'{dur:.0f} s of flight; the depth runs at ~15 Hz)', '',
         '| points >= | measurements in flight [Hz] | RMS error [m] | median abs [m] | 95% abs [m] | bias [m] | '
         'errors > 0.3 m |', '|---|---|---|---|---|---|---|']
    for k in MIN_POINTS:
        sel = fly & (f[:, 2] >= k)
        if sel.sum() == 0:
            continue
        e = err[sel]
        L.append(f'| {k} | {sel.sum() / dur:.1f} | {np.sqrt(np.mean(e ** 2)):.3f} | {np.median(np.abs(e)):.3f} | '
                 f'{np.percentile(np.abs(e), 95):.3f} | {np.mean(e):+.3f} | {np.mean(np.abs(e) > 0.3) * 100:.1f}% |')
    open(os.path.join(args.out, 'floor.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))
    t0 = t_fly[0] if len(t_fly) else f[0, 0]
    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    sc = ax[0].scatter(f[:, 0] - t0, err, c=np.minimum(f[:, 2], 1000), s=4, cmap='viridis')
    fig.colorbar(sc, ax=ax[0], label='points')
    ax[0].set_ylim(-0.5, 0.5), ax[0].set_ylabel('measured - true height [m]'), ax[0].grid(alpha=0.3)
    ax[1].plot(g[:, 0] - t0, g[:, 1] - args.floor_z, 'k', lw=1, label='true height')
    ax[1].plot(f[:, 0] - t0, f[:, 1], '.', ms=2, label='measured')
    ax[1].set_ylabel('height above floor [m]'), ax[1].set_xlabel('time since takeoff [s]'), ax[1].grid(alpha=0.3)
    ax[1].legend()
    ax[0].set_title('Floor height vs ground truth')
    fig.savefig(os.path.join(args.out, 'floor.png'), dpi=110, bbox_inches='tight')


if __name__ == '__main__':
    main()
