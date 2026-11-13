#!/usr/bin/env python3
"""Side-by-side comparison of the visual odometry (vo.tum) of several runs on the same flight.

    python3 compare.py <out_dir> <run_dir> [<run_dir> ...]

Writes <out_dir>/compare.md and compare.png: RPE, drift, resets (from each run's metrics.json, see
evaluate.py) and the yaw error added by every turn of the flight (ground-truth yaw rate above
TURN_RATE): the turns are where visual-only odometry loses heading.
"""

import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from evaluate import (ALIGN_FIRST, RPE_DIST, Traj, align_4dof, interp_gt, vo_segments, wrap,  # noqa: E402
                      yaw_of)

TURN_RATE = np.radians(10.0)   # rad/s: ground-truth yaw rate above this = turning
TURN_MIN = np.radians(45.0)    # turns smaller than this are ignored
MARGIN = 1.0                   # s before / after the turn where the yaw error is read


def turns(gt, t0, t1):
    m = (gt.t >= t0) & (gt.t <= t1)
    t, yaw = gt.t[m], np.unwrap(yaw_of(gt.rot[np.flatnonzero(m)]))
    rate = np.gradient(yaw, t)
    # smooth over 0.5 s, then runs above the threshold
    k = max(1, int(0.5 / np.median(np.diff(t))))
    rate = np.convolve(rate, np.ones(k) / k, mode='same')
    on = np.abs(rate) > TURN_RATE
    edges = np.flatnonzero(np.diff(on.astype(int)))
    out = []
    starts = edges[::2] + 1 if on[0] == 0 else np.r_[0, edges[1::2] + 1]
    for s in starts:
        e = s
        while e < len(on) and on[e]:
            e += 1
        e = min(e, len(t) - 1)
        if abs(yaw[e] - yaw[s]) >= TURN_MIN:
            out.append((t[s], t[e], np.degrees(yaw[e] - yaw[s])))
    return out


def yaw_error_curves(run):
    """per vo segment: times and yaw error (deg), aligned on the first ALIGN_FIRST s of the segment"""
    gt = Traj.load(os.path.join(run, 'gt.tum'))
    vo = Traj.load(os.path.join(run, 'vo.tum'))
    lost = np.array([])
    curves = []
    for s in vo_segments(vo, lost):
        if len(s) < 10:
            continue
        s = s.sel((s.t >= gt.t[0]) & (s.t <= gt.t[-1]))
        g = interp_gt(gt, s.t)
        a = align_4dof(s, g, s.t <= s.t[0] + ALIGN_FIRST)
        curves.append((s.t, np.degrees(wrap(yaw_of(a.rot) - yaw_of(g.rot)))))
    return gt, curves


def error_at(curves, t):
    for ts, e in curves:
        if ts[0] <= t <= ts[-1]:
            return float(np.interp(t, ts, e)), id(ts)
    return None, None


def main():
    out_dir, runs = sys.argv[1], sys.argv[2:]
    os.makedirs(out_dir, exist_ok=True)
    data = []
    for run in runs:
        m = json.load(open(os.path.join(run, 'metrics.json')))
        gt, curves = yaw_error_curves(run)
        name = os.path.basename(os.path.normpath(run))
        data.append((name, m, gt, curves))

    t_takeoff, t_end = data[0][1]['t_takeoff'], data[0][1]['t_end']
    tl = turns(data[0][2], t_takeoff, t_end)

    L = ['# Visual odometry comparison', '', f'flight: {data[0][1]["gt_distance_m"]:.0f} m, '
         f'{data[0][1]["flight_time_s"]:.0f} s, {len(tl)} turns', '',
         '| run | RPE 5 m | RPE 10 m | RPE 25 m | RPE 50 m | resets | ATE RMSE longest segment [m] | '
         'drift longest segment [% dist] | mean abs yaw error per turn [deg] |',
         '|---|---|---|---|---|---|---|---|---|']
    per_turn = {}
    for name, m, _, curves in data:
        vo = m['results']['vo']
        rp = vo['rpe_pooled']
        seg = max(vo['evaluated_segments'], key=lambda r: r['distance_m'])
        dy = []
        for a, b, _ in tl:
            ea, ia = error_at(curves, a - MARGIN)
            eb, ib = error_at(curves, b + MARGIN)
            dy.append(eb - ea if ea is not None and eb is not None and ia == ib else np.nan)
        per_turn[name] = dy
        cells = [f'{rp[f"{D:g}m"]["trans_pct_mean"]:.2f}% / {rp[f"{D:g}m"]["yaw_deg_mean"]:.2f}°'
                 if f'{D:g}m' in rp else '-' for D in RPE_DIST]
        L.append(f'| {name} | ' + ' | '.join(cells) + f' | {vo["resets_in_flight"]} | {seg["ate_rmse_m"]:.2f} '
                 f'({seg["distance_m"]:.0f} m) | {seg["drift_final_pct"]:.2f} | {np.nanmean(np.abs(dy)):.2f} |')
    L += ['', 'Yaw error added by each turn (error after - before, deg; "-" = reset during the turn):', '',
          '| turn | t [s] | angle [deg] | ' + ' | '.join(n for n, *_ in data) + ' |',
          '|---|---|---|' + '---|' * len(data)]
    for i, (a, b, ang) in enumerate(tl):
        L.append(f'| {i} | {a - t_takeoff:.0f} | {ang:+.0f} | ' + ' | '.join(
            '-' if np.isnan(per_turn[n][i]) else f'{per_turn[n][i]:+.2f}' for n, *_ in data) + ' |')
    open(os.path.join(out_dir, 'compare.md'), 'w').write('\n'.join(L) + '\n')
    print('\n'.join(L))

    fig, ax = plt.subplots(figsize=(12, 5))
    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    for k, (name, _, _, curves) in enumerate(data):
        for j, (ts, e) in enumerate(curves):
            ax.plot(ts - t_takeoff, e, color=colors[k % len(colors)], label=name if j == 0 else None)
    for a, b, _ in tl:
        ax.axvspan(a - t_takeoff, b - t_takeoff, color='0.85', zorder=0)
    ax.set_xlabel('time since takeoff [s]'), ax.set_ylabel('yaw error [deg]')
    ax.set_title('Visual odometry yaw error (grey = turns; a new line = reset)')
    ax.grid(alpha=0.3), ax.legend()
    fig.savefig(os.path.join(out_dir, 'compare.png'), dpi=120, bbox_inches='tight')


if __name__ == '__main__':
    main()
