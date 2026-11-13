#!/usr/bin/env bash
# Cross-check of evaluate.py with evo (standard tool) on the TUM files of a run: APE with SE(3)
# alignment on the whole trajectory, RPE over 10 m of path. Output in <run>/evo.txt.
# Uso: evo_check.sh /ws/images/odom_eval/<run>
RUN_DIR="${1:?run dir}"
cd "$RUN_DIR" || exit 1
command -v evo_ape >/dev/null || { echo "evo non installato (immagine warehouse-drone:humble da ricostruire)"; exit 0; }
{
  for est in vo ekf slam slam_final; do
    [ -s "$est.tum" ] || continue
    echo "=== $est: APE (translation, SE3 alignment) ==="
    # gt a 50 Hz: 20 ms bastano per associare ogni posa stimata a un campione di ground truth
    evo_ape tum gt.tum "$est.tum" -a --t_max_diff 0.02 --no_warnings 2>&1 | grep -E 'rmse|mean|max|compared'
    echo "=== $est: RPE over 10 m (translation) ==="
    evo_rpe tum gt.tum "$est.tum" -a --t_max_diff 0.02 --delta 10 --delta_unit m --all_pairs --no_warnings 2>&1 \
      | grep -E 'rmse|mean|max|compared'
  done
} | tee evo.txt
