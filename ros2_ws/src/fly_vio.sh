#!/usr/bin/env bash
# Da lanciare DENTRO il container dev (dopo scripts/start.sh start --vio): build del workspace,
# source e volo senza GPS ne' bussola sulla traiettoria predefinita, con la pipeline di odometria attuale:
# OpenVINS (coppia stereo sintetizzata + IMU della ZED) -> PX4 (EKF2 con drag), prua corretta con la
# struttura del magazzino (manhattan_yaw), SLAM RTAB-Map con correzione dei setpoint.
# Uso: /ws/src/fly_vio.sh [argomenti ros2 launch...]
#   es.  /ws/src/fly_vio.sh rviz:=true          (finestra RViz2 con la mappa)
#        /ws/src/fly_vio.sh manhattan:=false    (senza correzione della prua, per confronto)
# Gli argomenti passati si aggiungono ai default e sovrascrivono quelli con lo stesso nome.
set -e
cd /ws
colcon build --symlink-install
. install/setup.sh
exec ros2 launch daedalus_trajectories trajectory_following.launch.py \
  slam:=true odom:=openvins camera_noise:=true slam_correction:=true rviz:=false rtabmap_viz:=false "$@"
