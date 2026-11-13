#!/usr/bin/env python3
"""Odometry accuracy against Gazebo ground truth, from the TUM files of record_trajectories.py.

    python3 evaluate.py /ws/images/odom_eval/<run> [--label "text"]

Writes metrics.json, summary.md and plots (*.png) in the same folder.

Every estimate is aligned to the ground truth with x, y, z + yaw only (4 DoF): with an IMU roll and
pitch are observable, so they are not aligned away (the convention of VIO benchmarks, e.g. Zhang &
Scaramuzza 2018, "posyaw").
  ATE    RMSE of the position error, aligned on the whole trajectory (global consistency)
  drift  aligned only on the first ALIGN_FIRST s of flight, then left free: final error and
         error per travelled metre (what a controller flying on that estimate would see)
  RPE    relative error over sub-paths of D m of travelled distance (local accuracy, KITTI style):
         translation in % of D, yaw in deg
rgbd_odometry (vo) restarts from scratch after every tracking loss: it is split into segments at
every reset and each segment is evaluated on its own.
"""

import argparse
import json
import os

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ALIGN_FIRST = 15.0                 # s of flight used for the drift alignment
RPE_DIST = (5.0, 10.0, 25.0, 50.0)  # m
VO_JUMP = 0.5                      # m between consecutive vo poses = reset
VO_GAP = 1.0                       # s without vo poses = lost
MIN_SEG = 5.0                      # m: shorter vo segments are skipped
NAMES = {'ekf': 'PX4 EKF2 (/odom)', 'slam': 'SLAM online (map->base_link)',
         'slam_final': 'SLAM final graph (rtabmap.db)', 'vo': 'rgbd_odometry (/vo/odom)'}
COLORS = {'gt': 'k', 'ekf': 'tab:blue', 'slam': 'tab:green', 'slam_final': 'tab:red', 'vo': 'tab:orange'}


class Traj:
    def __init__(self, t, p, rot):
        self.t, self.p, self.rot = t, p, rot

    @staticmethod
    def load(path):
        if not os.path.exists(path) or os.path.getsize(path) == 0:
            return None
        d = np.atleast_2d(np.loadtxt(path))
        t, idx = np.unique(d[:, 0], return_index=True)
        d = d[idx]
        return Traj(t, d[:, 1:4], R.from_quat(d[:, 4:8]))

    def sel(self, mask):
        return Traj(self.t[mask], self.p[mask], self.rot[np.flatnonzero(mask)])

    def __len__(self):
        return len(self.t)


def interp_gt(gt, t):
    p = np.column_stack([np.interp(t, gt.t, gt.p[:, k]) for k in range(3)])
    return Traj(t, p, Slerp(gt.t, gt.rot)(t))


def yaw_of(rot):
    return rot.as_euler('ZYX')[:, 0]


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def align_4dof(est, gt, mask=None):
    """yaw, t such that Rz(yaw) * p_est + t ~ p_gt (least squares on the masked samples)"""
    m = np.ones(len(est), bool) if mask is None else mask
    pe, pg = est.p[m], gt.p[m]
    ce, cg = pe.mean(0), pg.mean(0)
    a, b = pe - ce, pg - cg
    yaw = np.arctan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    if m.sum() < 3 or np.linalg.norm(a[:, :2]) < 0.5:
        # (almost) no horizontal motion: yaw from the orientations
        yaw = np.angle(np.mean(np.exp(1j * wrap(yaw_of(gt.rot[np.flatnonzero(m)]) - yaw_of(est.rot[np.flatnonzero(m)])))))
    rz = R.from_euler('z', yaw)
    return Traj(est.t, rz.apply(est.p) + (cg - rz.apply(ce)), rz * est.rot)


def path_length(gt):
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt.p, axis=0), axis=1))])


def rpe(est, gt, dist):
    """translation errors (m) and yaw errors (rad) over sub-paths of `dist` m, one start per metre"""
    s = path_length(gt)
    if s[-1] < dist:
        return np.array([]), np.array([])
    starts = np.searchsorted(s, np.arange(0.0, s[-1] - dist, 1.0))
    ends = np.searchsorted(s, s[starts] + dist)
    ok = ends < len(s)
    starts, ends = starts[ok], ends[ok]
    et, ey = [], []
    for i, j in zip(starts, ends):
        # relative motion i -> j, in the frame of pose i
        dg = gt.rot[i].inv().apply(gt.p[j] - gt.p[i])
        de = est.rot[i].inv().apply(est.p[j] - est.p[i])
        rg = gt.rot[i].inv() * gt.rot[j]
        re = est.rot[i].inv() * est.rot[j]
        err_rot = rg.inv() * re
        et.append(np.linalg.norm(rg.inv().apply(de - dg)))
        ey.append(abs(err_rot.as_euler('ZYX')[0]))
    return np.array(et), np.array(ey)


def vo_segments(vo, lost):
    cut = np.flatnonzero((np.linalg.norm(np.diff(vo.p, axis=0), axis=1) > VO_JUMP) | (np.diff(vo.t) > VO_GAP)) + 1
    bounds = np.concatenate([[0], cut, [len(vo)]])
    segs = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        m = np.zeros(len(vo), bool)
        m[a:b] = True
        segs.append(vo.sel(m))
    return segs


def evaluate_traj(est, gt_full, t0_flight, absolute=False):
    gt = interp_gt(gt_full, est.t)
    out = {'samples': len(est), 'duration_s': float(est.t[-1] - est.t[0]),
           'distance_m': float(path_length(gt)[-1])}
    curves = {}
    if absolute:
        # no alignment: odom/map are ENU with the origin at the spawn (= first ground-truth pose), the
        # frame the planner's targets are given in. This is the error the drone flies with
        aerr = np.linalg.norm(est.p - (gt.p - gt_full.p[0]), axis=1)
        ayaw = wrap(yaw_of(est.rot) - yaw_of(gt.rot))
        out['abs_rmse_m'] = float(np.sqrt(np.mean(aerr ** 2)))
        out['abs_max_m'] = float(aerr.max())
        out['abs_final_m'] = float(aerr[-1])
        out['abs_yaw_mean_deg'] = float(np.degrees(np.angle(np.mean(np.exp(1j * ayaw)))))
        curves['abs_err'] = aerr
    # ATE: alignment on everything
    a = align_4dof(est, gt)
    err = np.linalg.norm(a.p - gt.p, axis=1)
    out['ate_rmse_m'] = float(np.sqrt(np.mean(err ** 2)))
    out['ate_max_m'] = float(err.max())
    # drift: alignment on the first ALIGN_FIRST s of flight only
    first = (est.t >= t0_flight) & (est.t <= t0_flight + ALIGN_FIRST)
    if first.sum() < 5:
        first = est.t <= est.t[0] + ALIGN_FIRST
    d = align_4dof(est, gt, first)
    derr = np.linalg.norm(d.p - gt.p, axis=1)
    yerr = wrap(yaw_of(d.rot) - yaw_of(gt.rot))
    out['drift_final_m'] = float(derr[-1])
    out['drift_max_m'] = float(derr.max())
    out['drift_final_pct'] = float(100 * derr[-1] / max(out['distance_m'], 1e-6))
    out['z_err_final_m'] = float(d.p[-1, 2] - gt.p[-1, 2])
    out['yaw_err_final_deg'] = float(np.degrees(yerr[-1]))
    out['yaw_err_max_deg'] = float(np.degrees(np.abs(yerr).max()))
    rp = {}
    for D in RPE_DIST:
        et, ey = rpe(est, gt, D)
        if len(et):
            rp[f'{D:g}m'] = {'n': int(len(et)), 'trans_pct_mean': float(100 * et.mean() / D),
                            'trans_pct_rmse': float(100 * np.sqrt(np.mean(et ** 2)) / D),
                            'yaw_deg_mean': float(np.degrees(ey.mean()))}
    out['rpe'] = rp
    curves.update({'t': est.t, 'ate_err': err, 'drift_err': derr, 'yaw_err': np.degrees(yerr),
                   'aligned_ate': a, 'aligned_drift': d, 'gt': gt})
    return out, curves


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--label', default='')
    args = ap.parse_args()
    run = args.run_dir
    meta = json.load(open(os.path.join(run, 'meta.json'))) if os.path.exists(os.path.join(run, 'meta.json')) else {}

    # mode = <closed_loop|shadow|replay>[:<estimator>] (record_trajectories.py --mode)
    mode, _, estimator = meta.get('mode', 'closed_loop').partition(':')
    if estimator == 'openvins':
        NAMES['vo'] = 'OpenVINS (/vo/odom)'
    if mode == 'shadow':
        # PX4 flies on GPS: EKF2 and SLAM are GPS-based references, only the vision estimators are under test
        NAMES['ekf'] = 'PX4 EKF2 (/odom, GPS)'
        NAMES['slam'] = 'SLAM online (on GPS odom)'
        NAMES['slam_final'] = 'SLAM final graph (on GPS odom)'
    gt = Traj.load(os.path.join(run, 'gt.tum'))
    t_takeoff = meta.get('t_takeoff') or gt.t[0]
    t_end = meta.get('t_crash') or gt.t[-1]
    t_start = t_takeoff - 3.0                         # the evaluation starts just before takeoff
    lost = np.loadtxt(os.path.join(run, 'vo_lost.txt'), ndmin=1) if os.path.getsize(os.path.join(run, 'vo_lost.txt')) else np.array([])

    def crop(tr):
        if tr is None:
            return None
        m = (tr.t >= max(t_start, gt.t[0])) & (tr.t <= min(t_end, gt.t[-1]))
        return tr.sel(m) if m.sum() > 10 else None

    results, curves = {}, {}
    for k in ('ekf', 'slam', 'slam_final'):
        tr = crop(Traj.load(os.path.join(run, f'{k}.tum')))
        if tr is not None:
            results[k], curves[k] = evaluate_traj(tr, gt, t_takeoff, absolute=True)

    vo = crop(Traj.load(os.path.join(run, 'vo.tum')))
    vo_res = None
    if vo is not None:
        segs = [s for s in vo_segments(vo, lost) if len(s) > 10]
        seg_res, seg_curves = [], []
        for s in segs:
            r, c = evaluate_traj(s, gt, t_takeoff if s.t[0] <= t_takeoff else s.t[0])
            if r['distance_m'] >= MIN_SEG:
                seg_res.append(r)
                seg_curves.append(c)
        lost_flight = lost[(lost >= t_takeoff) & (lost <= t_end)] if len(lost) else lost
        vo_res = {'segments': len(segs), 'resets_in_flight': max(len(segs) - 1, 0),
                  'lost_frames_in_flight': int(len(lost_flight)),
                  'evaluated_segments': seg_res}
        # RPE pooled over all segments (weighted by number of sub-paths)
        pooled = {}
        for D in RPE_DIST:
            key = f'{D:g}m'
            ns = [(r['rpe'][key]['n'], r['rpe'][key]) for r in seg_res if key in r['rpe']]
            if ns:
                n = sum(x for x, _ in ns)
                pooled[key] = {'n': n,
                               'trans_pct_mean': sum(x * v['trans_pct_mean'] for x, v in ns) / n,
                               'yaw_deg_mean': sum(x * v['yaw_deg_mean'] for x, v in ns) / n}
        vo_res['rpe_pooled'] = pooled
        results['vo'] = vo_res
        curves['vo'] = seg_curves

    gt_f = gt.sel((gt.t >= t_start) & (gt.t <= t_end))
    summary = {'run': run, 'label': args.label, 'end': meta.get('end'), 't_takeoff': t_takeoff, 't_end': t_end,
               'flight_time_s': float(t_end - t_takeoff), 'gt_distance_m': float(path_length(gt_f)[-1]),
               'results': results}
    with open(os.path.join(run, 'metrics.json'), 'w') as f:
        json.dump(summary, f, indent=2)

    # ---------------- summary.md
    L = [f'# Odometry evaluation {args.label}', '',
         f'- run: `{run}`', f'- mode: {meta.get("mode", "closed_loop")}', f'- end: {meta.get("end")}',
         f'- flight time: {summary["flight_time_s"]:.0f} s, ground-truth distance: {summary["gt_distance_m"]:.1f} m', '',
         '**Absolute error** (no alignment: the error the drone flies with, frame = spawn pose, ENU)', '',
         '| estimate | RMSE [m] | max [m] | final [m] | mean yaw offset [deg] |', '|---|---|---|---|---|']
    for k in ('ekf', 'slam', 'slam_final'):
        if k in results:
            r = results[k]
            L.append(f'| {NAMES[k]} | {r["abs_rmse_m"]:.2f} | {r["abs_max_m"]:.2f} | {r["abs_final_m"]:.2f} | '
                     f'{r["abs_yaw_mean_deg"]:+.2f} |')
    L += ['', '**Odometry quality** (4-DoF aligned: ATE on the whole flight, drift after aligning the first '
          f'{ALIGN_FIRST:g} s)', '',
          '| estimate | ATE RMSE [m] | ATE max [m] | final drift [m] | drift [% dist] | max drift [m] | final yaw err [deg] | final z err [m] |',
         '|---|---|---|---|---|---|---|---|']
    for k in ('ekf', 'slam', 'slam_final'):
        if k in results:
            r = results[k]
            L.append(f'| {NAMES[k]} | {r["ate_rmse_m"]:.2f} | {r["ate_max_m"]:.2f} | {r["drift_final_m"]:.2f} | '
                     f'{r["drift_final_pct"]:.2f} | {r["drift_max_m"]:.2f} | {r["yaw_err_final_deg"]:+.2f} | {r["z_err_final_m"]:+.2f} |')
    L += ['', '**RPE** (relative error over D m of path: translation % / yaw deg)', '',
          '| estimate | ' + ' | '.join(f'{D:g} m' for D in RPE_DIST) + ' |', '|---|' + '---|' * len(RPE_DIST)]
    for k in ('ekf', 'slam', 'slam_final', 'vo'):
        if k not in results:
            continue
        rp = results[k]['rpe_pooled'] if k == 'vo' else results[k]['rpe']
        cells = [f'{rp[f"{D:g}m"]["trans_pct_mean"]:.2f}% / {rp[f"{D:g}m"]["yaw_deg_mean"]:.2f}°'
                 if f'{D:g}m' in rp else '-' for D in RPE_DIST]
        L.append(f'| {NAMES[k]} | ' + ' | '.join(cells) + ' |')
    if vo_res:
        L += ['', f'**{NAMES["vo"]}**: {vo_res["resets_in_flight"]} resets in flight, '
              f'{vo_res["lost_frames_in_flight"]} lost frames; segments >= {MIN_SEG:g} m:', '',
              '| segment | length [m] | duration [s] | ATE RMSE [m] | drift [% dist] |', '|---|---|---|---|---|']
        for i, r in enumerate(vo_res['evaluated_segments']):
            L.append(f'| {i} | {r["distance_m"]:.1f} | {r["duration_s"]:.0f} | {r["ate_rmse_m"]:.2f} | {r["drift_final_pct"]:.2f} |')
    open(os.path.join(run, 'summary.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))

    # ---------------- plots
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(gt_f.p[:, 0], gt_f.p[:, 1], COLORS['gt'], lw=2, label='ground truth')
    for k in ('ekf', 'slam', 'slam_final'):
        if k in curves:
            a = curves[k]['aligned_drift']
            ax.plot(a.p[:, 0], a.p[:, 1], color=COLORS[k], lw=1, label=NAMES[k])
    for i, c in enumerate(curves.get('vo', [])):
        a = c['aligned_drift']
        ax.plot(a.p[:, 0], a.p[:, 1], color=COLORS['vo'], lw=0.8, ls='--', label=NAMES['vo'] + ' (segments)' if i == 0 else None)
    ax.set_aspect('equal'), ax.grid(alpha=0.3), ax.legend(fontsize=8)
    ax.set_xlabel('x [m]'), ax.set_ylabel('y [m]')
    ax.set_title(f'Trajectories, aligned on the first {ALIGN_FIRST:g} s of flight {args.label}')
    fig.savefig(os.path.join(run, 'traj_xy.png'), dpi=130, bbox_inches='tight')

    fig, axs = plt.subplots(4, 1, figsize=(11, 13), sharex=True)
    for k in ('ekf', 'slam', 'slam_final'):
        if k in curves:
            c = curves[k]
            axs[0].plot(c['t'] - t_takeoff, c['drift_err'], color=COLORS[k], label=NAMES[k])
            axs[1].plot(c['t'] - t_takeoff, c['yaw_err'], color=COLORS[k])
            axs[2].plot(c['t'] - t_takeoff, c['aligned_drift'].p[:, 2] - c['gt'].p[:, 2], color=COLORS[k])
            axs[3].plot(c['t'] - t_takeoff, c['abs_err'], color=COLORS[k])
    for i, c in enumerate(curves.get('vo', [])):
        axs[0].plot(c['t'] - t_takeoff, c['drift_err'], color=COLORS['vo'], ls='--', label=NAMES['vo'] if i == 0 else None)
        axs[1].plot(c['t'] - t_takeoff, c['yaw_err'], color=COLORS['vo'], ls='--')
    if len(lost):
        for t in lost[(lost >= t_start) & (lost <= t_end)]:
            axs[0].axvline(t - t_takeoff, color='tab:orange', alpha=0.15, lw=0.8)
    axs[0].set_ylabel('position error [m]'), axs[1].set_ylabel('yaw error [deg]'), axs[2].set_ylabel('z error [m]')
    axs[3].set_ylabel('absolute error [m]\n(no alignment)')
    axs[3].set_xlabel('time since takeoff [s]')
    for a in axs:
        a.grid(alpha=0.3)
    axs[0].legend(fontsize=8)
    axs[0].set_title(f'Errors vs ground truth (drift alignment; orange lines = vo lost frames) {args.label}')
    fig.savefig(os.path.join(run, 'errors.png'), dpi=130, bbox_inches='tight')
    print(f'\nplots and metrics in {run}')


if __name__ == '__main__':
    main()
