#!/usr/bin/env bash
# Da lanciare DENTRO il container dev (dopo scripts/start.sh start --no-gps): build del workspace,
# source e volo senza GPS con traiettoria + SLAM + camera rumorosa (odometria visuale -> PX4).
# Uso: /ws/src/fly_no_gps.sh [argomenti ros2 launch...]
#   es.  /ws/src/fly_no_gps.sh rviz:=false camera_noise:=false
# Gli argomenti passati si aggiungono ai default e sovrascrivono quelli con lo stesso nome.
# POSIX (". setup.sh" invece di "source setup.bash"): funziona anche con "sh fly_no_gps.sh"
set -e
cd /ws
colcon build --symlink-install
. install/setup.sh
exec ros2 launch daedalus_trajectories trajectory_following.launch.py \
  slam:=true odom:=visual camera_noise:=true rviz:=true slam_correction:=true "$@"
