#!/usr/bin/env python3
"""Accuracy of the Manhattan-world heading (/manhattan/yaw, daedalus_slam/manhattan_yaw) against the
ground truth (/gt/odom), on a bag replay (odom_replay.launch.py starts manhattan_yaw):

    ros2 launch daedalus_slam odom_replay.launch.py estimator:=openvins &
    python3 eval_manhattan.py --out /ws/images/odom_eval/<run> &
    ros2 bag play /ws/images/odom_bags/<bag>

Expected value: the warehouse axes are at `--axes` deg (ENU yaw, 0 = the world's x/y axes), so a drone with
true ENU yaw psi should measure wrap90(axes - psi). Writes manhattan.md (error and availability per
quality threshold, also without the turns) and manhattan.png.
"""

import argparse
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from daedalus_interfaces.msg import ManhattanYaw
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

QUALITIES = (0.0, 0.3, 0.5, 0.7, 0.8)
TURN_RATE = 10.0     # deg/s: true yaw rate above this = turning


def wrap90(a):
    """angle (rad) -> [-pi/4, pi/4)"""
    return (a + np.pi / 4) % (np.pi / 2) - np.pi / 4


class Rec(Node):
    def __init__(self):
        super().__init__('eval_manhattan', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.m, self.g = [], []
        self.create_subscription(ManhattanYaw, '/manhattan/yaw', self.on_m, 50)
        self.create_subscription(Odometry, '/gt/odom', self.on_g, 200)

    def on_m(self, msg):
        self.m.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.yaw, msg.quality, msg.num_normals))

    def on_g(self, msg):
        q = msg.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.g.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, yaw, msg.pose.pose.position.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--axes', type=float, default=0.0, help='ENU yaw (deg) of the warehouse axes')
    ap.add_argument('--stall', type=float, default=5.0, help='wall s without messages = end of the bag')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    n = Rec()
    last, last_wall = 0, time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if len(n.m) != last:
            last, last_wall = len(n.m), time.monotonic()
        elif last and time.monotonic() - last_wall > args.stall:
            break
    m, g = np.array(n.m), np.array(sorted(n.g))
    rclpy.try_shutdown()

    yaw_g = np.unwrap(g[:, 1])
    rate = np.degrees(np.gradient(yaw_g, g[:, 0]))
    psi = np.interp(m[:, 0], g[:, 0], yaw_g)
    turning = np.abs(np.interp(m[:, 0], g[:, 0], rate)) > TURN_RATE
    flying = np.interp(m[:, 0], g[:, 0], g[:, 2] - g[0, 2]) > 1.0
    expected = wrap90(np.radians(args.axes) - psi)
    err = np.degrees(wrap90(m[:, 1] - expected))

    L = ['# Manhattan heading accuracy', '', f'{len(m)} measurements, {flying.sum()} in flight, '
         f'{(turning & flying).sum()} while turning (> {TURN_RATE:g} deg/s)', '',
         '| quality >= | availability in flight | RMS error [deg] | median abs [deg] | 95% abs [deg] | bias [deg] | '
         'RMS not turning [deg] |', '|---|---|---|---|---|---|---|']
    for qmin in QUALITIES:
        sel = flying & (m[:, 2] >= qmin)
        st = sel & ~turning
        if sel.sum() == 0:
            continue
        e = err[sel]
        L.append(f'| {qmin:.1f} | {100 * sel.sum() / flying.sum():.0f}% | {np.sqrt(np.mean(e ** 2)):.2f} | '
                 f'{np.median(np.abs(e)):.2f} | {np.percentile(np.abs(e), 95):.2f} | {np.mean(e):+.2f} | '
                 f'{np.sqrt(np.mean(err[st] ** 2)):.2f} |')
    L += ['', f'normals per measurement: median {np.median(m[:, 3]):.0f}, 5% {np.percentile(m[:, 3], 5):.0f}']
    open(os.path.join(args.out, 'manhattan.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))
    np.savetxt(os.path.join(args.out, 'manhattan.txt'), np.column_stack([m, np.degrees(expected), err]),
               header='t yaw quality num_normals expected_deg err_deg')

    t0 = g[np.argmax(g[:, 2] - g[0, 2] > 1.0), 0]
    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    sc = ax[0].scatter(m[:, 0] - t0, err, c=m[:, 2], s=4, cmap='viridis', vmin=0, vmax=1)
    ax[0].set_ylabel('error [deg]'), ax[0].set_ylim(-10, 10), ax[0].grid(alpha=0.3)
    fig.colorbar(sc, ax=ax[0], label='quality')
    ax[1].plot(m[:, 0] - t0, m[:, 2], '.', ms=2)
    ax[1].set_ylabel('quality'), ax[1].set_xlabel('time since takeoff [s]'), ax[1].grid(alpha=0.3)
    for a in ax:
        for i in np.flatnonzero(turning & flying)[::3]:
            a.axvline(m[i, 0] - t0, color='0.9', zorder=0)
    ax[0].set_title('Manhattan heading error vs ground truth (grey = turning)')
    fig.savefig(os.path.join(args.out, 'manhattan.png'), dpi=110, bbox_inches='tight')


if __name__ == '__main__':
    main()
