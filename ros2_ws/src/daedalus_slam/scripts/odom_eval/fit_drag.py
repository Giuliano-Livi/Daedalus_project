#!/usr/bin/env python3
"""Fits PX4's multirotor drag model (EKF2_DRAG_CTRL) on a recorded flight (run_eval.sh --bag).

    python3 fit_drag.py /ws/images/odom_bags/<bag> [--out <dir>]

The thrust of a multirotor is along body z, so the horizontal body specific force measured by the
accelerometer is the aerodynamic drag only. EKF2's model, per body axis i = x, y (no wind):
    a_i = -MCOEF * v_i - rho / (2 * BCOEF_i) * v_i * |v_i|
(momentum drag of the propellers + bluff-body drag), v_i = body-frame velocity relative to the air.
a_i: PX4's IMU (/imu/data, the accelerometer EKF2 uses); v_i: ground-truth velocity (/gt/odom), rotated
into the body frame. Both low-pass filtered over SMOOTH s (the fit is on the slow drag, not on the
noise / vibrations). Prints the EKF2 parameters and saves drag_fit.png.
"""

import argparse
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RHO = 1.225          # kg/m^3, air density used by EKF2 (sea level)
SMOOTH = 0.5         # s, moving average on acceleration and velocity
MIN_HEIGHT = 1.0     # m above the start: only the flight (no ground contact forces)


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/imu/data', '/gt/odom']))
    imu, gt = [], []
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/imu/data':
            m = deserialize_message(data, Imu)
            a = m.linear_acceleration
            imu.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, a.x, a.y, a.z))
        else:
            m = deserialize_message(data, Odometry)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            gt.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return np.array(sorted(imu)), np.array(sorted(gt))


def smooth(x, n):
    k = np.ones(n) / n
    return np.column_stack([np.convolve(c, k, mode='same') for c in np.atleast_2d(x.T)]) if x.ndim > 1 \
        else np.convolve(x, k, mode='same')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    out = args.out or os.path.join('/ws/images/odom_eval', 'drag_fit_' + os.path.basename(os.path.normpath(args.bag)))
    os.makedirs(out, exist_ok=True)

    imu, gt = read(args.bag)
    t, p, rot = gt[:, 0], gt[:, 1:4], R.from_quat(gt[:, 4:8])
    # world velocity from positions (central differences), on the IMU timestamps
    v_w = np.gradient(p, t, axis=0)
    ti = imu[:, 0]
    m = (ti > t[0]) & (ti < t[-1])
    ti, acc = ti[m], imu[m, 1:4]
    v_wi = np.column_stack([np.interp(ti, t, v_w[:, k]) for k in range(3)])
    z_i = np.interp(ti, t, p[:, 2])
    from scipy.spatial.transform import Slerp
    rot_i = Slerp(t, rot)(ti)
    v_b = rot_i.inv().apply(v_wi)                 # body-frame (FLU) velocity

    n = max(1, int(SMOOTH / np.median(np.diff(ti))))
    acc_s, v_s = smooth(acc, n), smooth(v_b, n)
    fly = (z_i - p[0, 2]) > MIN_HEIGHT
    fly[:n] = fly[-n:] = False

    res = {}
    fig, axs = plt.subplots(1, 2, figsize=(12, 5))
    for k, ax_name in enumerate('xy'):
        v, a = v_s[fly, k], acc_s[fly, k]
        # a = c1 * v + c2 * v|v|  (least squares, no offset: zero drag at zero speed)
        A = np.column_stack([v, v * np.abs(v)])
        (c1, c2), *_ = np.linalg.lstsq(A, a, rcond=None)
        lin, *_ = np.linalg.lstsq(v[:, None], a, rcond=None)
        resid = a - A @ np.array([c1, c2])
        res[ax_name] = (c1, c2, lin[0], np.sqrt(np.mean(resid ** 2)), np.sqrt(np.mean(a ** 2)))
        ax = axs[k]
        ax.plot(v[::20], a[::20], '.', ms=2, alpha=0.4, label='flight samples')
        vv = np.linspace(v.min(), v.max(), 100)
        ax.plot(vv, c1 * vv + c2 * vv * np.abs(vv), 'r', label=f'fit: {c1:.3f} v + {c2:.3f} v|v|')
        ax.set_xlabel(f'body {ax_name} velocity [m/s]'), ax.set_ylabel(f'body {ax_name} specific force [m/s^2]')
        ax.grid(alpha=0.3), ax.legend(fontsize=8)
    fig.suptitle('Drag fit on ' + os.path.basename(os.path.normpath(args.bag)))
    fig.savefig(os.path.join(out, 'drag_fit.png'), dpi=110, bbox_inches='tight')

    lines = ['# Drag fit', '', f'bag: {args.bag}, {fly.sum()} IMU samples in flight, smoothing {SMOOTH} s', '',
             '| axis | linear [1/s] | quadratic [1/m] | linear-only fit [1/s] | residual rms [m/s^2] | signal rms [m/s^2] |',
             '|---|---|---|---|---|---|']
    for ax_name, (c1, c2, l1, rr, sr) in res.items():
        lines.append(f'| {ax_name} | {c1:.4f} | {c2:.4f} | {l1:.4f} | {rr:.4f} | {sr:.4f} |')
    mcoef = -np.mean([res['x'][0], res['y'][0]])
    lines += ['', 'EKF2 parameters:', f'- EKF2_MCOEF = {mcoef:.3f} 1/s (mean of x, y)']
    for ax_name in 'xy':
        c2 = res[ax_name][1]
        bc = RHO / (2 * -c2) if c2 < -1e-4 else float('inf')
        lines.append(f'- EKF2_BCOEF_{ax_name.upper()} = {bc:.1f} kg/m^2' + (' (no measurable bluff-body drag)' if not np.isfinite(bc) else ''))
    open(os.path.join(out, 'drag_fit.md'), 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
