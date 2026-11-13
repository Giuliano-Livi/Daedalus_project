#!/usr/bin/env python3
"""Heading and height sent to PX4 by vision_to_px4 (/fmu/in/vehicle_visual_odometry) vs ground truth (/gt/odom), on a
bag replay: offline check of the Manhattan and floor corrections (vision_to_px4 mode vio, without PX4).

    python3 check_heading.py --out <dir> [--label text]
Writes heading.md and heading.png (heading error over time, applied corrections)."""
import argparse, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Vector3Stamped
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def wrap(a):
    return (a + 180) % 360 - 180


class Rec(Node):
    def __init__(self):
        super().__init__('check_heading', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.ev, self.gt, self.corr, self.fl = [], [], [], []
        self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.on_ev, 50)
        self.create_subscription(Odometry, '/gt/odom', self.on_gt, 200)
        self.create_subscription(Vector3Stamped, '/manhattan/correction', self.on_c, 50)
        self.create_subscription(Vector3Stamped, '/floor/correction', lambda m: self.fl.append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.vector.x, m.vector.y, m.vector.z)), 50)

    def on_ev(self, m):
        w, x, y, z = m.q   # NED, body FRD
        self.ev.append((m.timestamp_sample * 1e-6, np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))),
                        -m.position[2]))

    def on_gt(self, m):
        q = m.pose.pose.orientation
        psi = np.degrees(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.gt.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 90.0 - psi, m.pose.pose.position.z))

    def on_c(self, m):
        self.corr.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.vector.x, m.vector.y, m.vector.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--label', default='')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    n = Rec()
    last, wall = 0, time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if len(n.ev) != last:
            last, wall = len(n.ev), time.monotonic()
        elif last and time.monotonic() - wall > 5:
            break
    ev, gt, c = np.array(n.ev), np.array(sorted(n.gt)), np.array(n.corr) if n.corr else np.zeros((0, 4))
    fl = np.array(n.fl) if n.fl else np.zeros((0, 4))
    rclpy.try_shutdown()
    hg = np.degrees(np.unwrap(np.radians(gt[:, 1])))
    err = wrap(ev[:, 1] - np.interp(ev[:, 0], gt[:, 0], hg))
    t0 = gt[np.argmax(gt[:, 2] - gt[0, 2] > 1.0), 0]
    fly = ev[:, 0] > t0
    e = err[fly]
    L = [f'# Heading sent to PX4 vs ground truth {args.label}', '',
         f'in flight: mean {e.mean():+.2f} deg, RMS {np.sqrt(np.mean(e ** 2)):.2f} deg, max abs {np.abs(e).max():.2f} deg, '
         f'final {e[-1]:+.2f} deg']
    if len(c):
        L.append(f'manhattan: {int(c[:, 3].sum())} measurements used of {len(c)}, total correction {c[:, 2].sum():+.2f} deg')
    # height: sent to PX4 (up, origin at the start) vs ground truth (base_link height minus the start)
    ez = ev[:, 2] - (np.interp(ev[:, 0], gt[:, 0], gt[:, 2]) - gt[0, 2])
    z = ez[fly]
    L += ['', f'height in flight: mean {z.mean():+.3f} m, RMS {np.sqrt(np.mean(z ** 2)):.3f} m, max abs {np.abs(z).max():.3f} m, '
          f'final {z[-1]:+.3f} m']
    if len(fl):
        L.append(f'floor: {int(fl[:, 3].sum())} measurements used of {len(fl)}, total correction {fl[:, 2].sum():+.3f} m')
    open(os.path.join(args.out, 'heading.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))
    np.savetxt(os.path.join(args.out, 'heading.txt'), np.column_stack([ev[:, 0], err]), header='t err_deg')
    fig, (ax, az) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax.plot(ev[:, 0] - t0, err, label='heading sent to PX4 - ground truth')
    if len(c):
        ax.plot(c[:, 0] - t0, np.cumsum(c[:, 2]), label='cumulative Manhattan correction')
    ax.set_ylabel('deg'), ax.grid(alpha=0.3), ax.legend()
    ax.set_title(f'Heading and height error {args.label}')
    az.plot(ev[:, 0] - t0, ez, label='height sent to PX4 - ground truth')
    if len(fl):
        az.plot(fl[:, 0] - t0, np.cumsum(fl[:, 2]), label='cumulative floor correction')
    az.set_xlabel('time since takeoff [s]'), az.set_ylabel('m'), az.grid(alpha=0.3), az.legend()
    fig.savefig(os.path.join(args.out, 'heading.png'), dpi=110, bbox_inches='tight')


if __name__ == '__main__':
    main()
