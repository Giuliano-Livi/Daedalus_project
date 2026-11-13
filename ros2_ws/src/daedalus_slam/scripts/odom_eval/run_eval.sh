#!/usr/bin/env bash
# Flight on the predefined path + recording of ground truth and odometry, then evaluate.py (+ evo).
# Run inside the dev container.
# Uso: /ws/src/daedalus_slam/scripts/odom_eval/run_eval.sh [--shadow] [--bag] <run_name> [argomenti ros2 launch...]
#   (default)  closed loop: sim started with scripts/start.sh start --no-gps, PX4 flies on the visual odometry
#   --shadow   sim started WITH GPS (scripts/start.sh start): PX4 flies on GPS and ignores the vision
#              (EKF2_EV_CTRL = 0), the visual odometry runs in parallel and is only evaluated. Every
#              estimator sees the same, complete flight (no crash), so they can be compared
#   --bag      records the flight as a dataset (ideal images before camera_noise, IMU, camera_info,
#              static TF, ground truth /gt/odom) in /ws/images/odom_bags/<run_name>, for run_replay.sh
#   es.  run_eval.sh --shadow --bag shadow_rgbd
# Results in /ws/images/odom_eval/<run_name> (images/odom_eval on the host).
set -e
MODE=closed_loop; BAG=0
while [[ "$1" == --* ]]; do
  case "$1" in
    --shadow) MODE=shadow ;;
    --bag)    BAG=1 ;;
    *) echo "opzione sconosciuta: $1" >&2; exit 1 ;;
  esac
  shift
done
RUN="${1:?run name}"; shift
OUT="/ws/images/odom_eval/$RUN"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"
cd /ws
. install/setup.sh

PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
}
trap cleanup EXIT

if (( BAG )); then
  BAG_DIR="/ws/images/odom_bags/$RUN"
  rm -rf "$BAG_DIR"; mkdir -p "$(dirname "$BAG_DIR")"
  # immagini ideali (/sim/camera, prima di camera_noise): il rumore si riapplica nel replay; l'immagine
  # destra non serve, stereo_synth la rigenera dalle stesse colore + depth
  ros2 bag record -o "$BAG_DIR" \
    /clock /imu/data /zed/imu/data /gt/odom /drone/setpoint /tf_static \
    /sim/camera/color/image_raw /sim/camera/depth/image_raw \
    /camera/color/camera_info /camera/depth/camera_info \
    > "$OUT/bag.log" 2>&1 &
  PIDS+=($!)
  sleep 2
fi

if [[ "$MODE" == shadow ]]; then
  # stessa pipeline visiva del volo senza GPS, ma PX4 vola sul GPS: niente correzione SLAM dei setpoint
  LAUNCH_ARGS=(slam:=true odom:=visual camera_noise:=true rviz:=false rtabmap_viz:=false slam_correction:=false)
else
  LAUNCH_ARGS=(slam:=true odom:=visual camera_noise:=true rviz:=false rtabmap_viz:=false slam_correction:=true)
fi
ros2 launch daedalus_trajectories trajectory_following.launch.py "${LAUNCH_ARGS[@]}" "$@" \
  > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
PIDS+=($LAUNCH_PID)

# stimatore visivo (per le etichette di evaluate.py): rgbd_odometry salvo odom:=openvins negli argomenti
ESTIMATOR=rgbd
for a in "$@"; do [[ "$a" == odom:=openvins ]] && ESTIMATOR=openvins; done
python3 "$HERE/record_trajectories.py" --out "$OUT" --mode "$MODE:$ESTIMATOR" 2>&1 | tee "$OUT/recorder.log"

trap - EXIT
cleanup
# il launch a volte non si chiude con SIGINT (rtabmap): dopo 15 s si forza
for _ in $(seq 15); do kill -0 $LAUNCH_PID 2>/dev/null || break; sleep 1; done
pkill -KILL -f 'ros2 launch daedalus_trajectories' 2>/dev/null || true
pkill -KILL -f '/opt/ros/humble/lib/rtabmap' 2>/dev/null || true
pkill -KILL -f 'lib/daedalus_' 2>/dev/null || true
pkill -KILL -f 'parameter_bridge' 2>/dev/null || true
pkill -KILL -f 'run_subscribe_msckf' 2>/dev/null || true
wait 2>/dev/null || true
# grafo ottimizzato finale (le pose su cui e' costruita la mappa), con i timestamp dei nodi
DB=/root/.ros/rtabmap.db
if [ -f "$DB" ]; then
  (cd "$OUT" && cp "$DB" rtabmap.db && rtabmap-report --poses_raw rtabmap.db >/dev/null 2>&1 || true
   [ -f rtabmap_slam.txt ] && grep -v '^#' rtabmap_slam.txt | awk '{print $1,$2,$3,$4,$5,$6,$7,$8}' > slam_final.tum
   rm -f rtabmap.db rtabmap_slam.txt rtabmap_odom.txt rtabmap_gt.txt)
fi
python3 "$HERE/evaluate.py" "$OUT" --label "$RUN"
"$HERE/evo_check.sh" "$OUT"
