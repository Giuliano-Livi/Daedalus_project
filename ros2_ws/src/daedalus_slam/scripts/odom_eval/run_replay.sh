#!/usr/bin/env bash
# Replays a flight recorded with run_eval.sh --bag through an odometry estimator and evaluates it
# against the ground truth stored in the bag. No simulator needed: every estimator sees exactly the
# same data, so runs are comparable (only the random camera noise differs).
# Run inside the dev container.
# Uso: /ws/src/daedalus_slam/scripts/odom_eval/run_replay.sh <bag_name> <run_name> [estimator] [rate]
#   es.  run_replay.sh shadow_rgbd replay_rgbd rgbd
#   bag in /ws/images/odom_bags/<bag_name>, results in /ws/images/odom_eval/<run_name>
set -e
BAG="/ws/images/odom_bags/${1:?bag name}"
RUN="${2:?run name}"
ESTIMATOR="${3:-rgbd}"
RATE="${4:-1.0}"
OUT="/ws/images/odom_eval/$RUN"
HERE="$(cd "$(dirname "$0")" && pwd)"
[ -d "$BAG" ] || { echo "bag non trovato: $BAG" >&2; exit 1; }
mkdir -p "$OUT"
cd /ws
. install/setup.sh

ros2 launch daedalus_slam odom_replay.launch.py estimator:="$ESTIMATOR" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
python3 "$HERE/record_trajectories.py" --out "$OUT" --gt-source ros --mode "replay:$ESTIMATOR" \
  > "$OUT/recorder.log" 2>&1 &
REC_PID=$!
trap 'kill -INT $LAUNCH_PID $REC_PID 2>/dev/null || true' EXIT
sleep 3     # nodi pronti prima dei primi messaggi (tf_static, primi frame)

# /clock registrato = tempo di simulazione del volo
ros2 bag play "$BAG" --rate "$RATE" > "$OUT/play.log" 2>&1
wait $REC_PID || true      # si ferma da solo quando il tempo simulato non avanza piu'
trap - EXIT
kill -INT $LAUNCH_PID 2>/dev/null || true
for _ in $(seq 10); do kill -0 $LAUNCH_PID 2>/dev/null || break; sleep 1; done
pkill -KILL -f 'odom_replay.launch.py' 2>/dev/null || true
pkill -KILL -f '/opt/ros/humble/lib/rtabmap' 2>/dev/null || true
pkill -KILL -f 'lib/daedalus_camera' 2>/dev/null || true
pkill -KILL -f 'lib/daedalus_slam' 2>/dev/null || true
pkill -KILL -f 'run_subscribe_msckf' 2>/dev/null || true
tail -2 "$OUT/recorder.log"
python3 "$HERE/evaluate.py" "$OUT" --label "$RUN"
"$HERE/evo_check.sh" "$OUT"
