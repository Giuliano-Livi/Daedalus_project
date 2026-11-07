#!/usr/bin/env bash
# Capture the shelf datasets (scripts/capture_datasets.py) into images/datasets/:
# headless Gazebo (partition "dataset", does not disturb a running simulation) + ROS container with
# the bridge and the camera_noise nodes, one task after the other.
# Uso: scripts/capture_datasets.sh [freespot] [inventory]   (default: both)
set -e
cd "$(dirname "$0")/.."
OUT=images/datasets
TASKS=${*:-freespot inventory}
mkdir -p "$OUT"
cleanup() { docker rm -f ds_server ds_client >/dev/null 2>&1 || true; rm -f gazebo/dataset_capture_world.sdf; }
trap cleanup EXIT

for task in $TASKS; do
  python3 scripts/capture_datasets.py world "$task" gazebo/dataset_capture_world.sdf
  # whole gazebo/ folder mounted: gz sim refuses a single bind-mounted world file under snap docker
  docker run -d --rm --name ds_server --network host --ipc host --device /dev/dri \
    -e GZ_PARTITION=dataset -e GZ_SIM_RESOURCE_PATH=/g/models -v "$PWD/gazebo:/g:ro" \
    --entrypoint bash px4io/px4-sitl-gazebo:latest \
    -c 'gz sim -s -r --headless-rendering -v 2 /g/dataset_capture_world.sdf' >/dev/null
  sleep 25
  NOISE=$(python3 scripts/capture_datasets.py noise "$task")
  docker run --rm --name ds_client --network host --ipc host -e GZ_PARTITION=dataset \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds.xml -v "$PWD/docker/ros/fastdds.xml:/etc/fastdds.xml:ro" \
    -v "$PWD/ros2_ws/src:/ws/src" -v "$PWD/scripts:/repo/scripts:ro" -v "$PWD/gazebo:/repo/gazebo:ro" \
    -v "$PWD/$OUT:/out" --entrypoint bash warehouse-drone:humble -c "
    source /opt/ros/humble/setup.bash; source /deps_ws/install/setup.bash
    cd /ws && colcon build --symlink-install --packages-select daedalus_camera >/dev/null 2>&1; source install/setup.bash
    ros2 run ros_gz_bridge parameter_bridge \
      /dataset/camera/color/image@sensor_msgs/msg/Image[gz.msgs.Image \
      /dataset/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image \
      --ros-args -r /dataset/camera/color/image:=/dataset/camera/color/ideal \
      -r /dataset/camera/depth/image:=/dataset/camera/depth/ideal > /tmp/bridge.log 2>&1 &
    ros2 run daedalus_camera camera_noise --ros-args -r __node:=noise_color -p stream:=color \
      -r color_in:=/dataset/camera/color/ideal -r color_out:=/dataset/camera/color/noisy > /dev/null 2>&1 &
    ros2 run daedalus_camera camera_noise --ros-args -r __node:=noise_depth -p stream:=depth $NOISE \
      -r depth_in:=/dataset/camera/depth/ideal -r depth_out:=/dataset/camera/depth/noisy > /dev/null 2>&1 &
    python3 /repo/scripts/capture_datasets.py capture $task /out; status=\$?
    kill %1 %2 %3 2>/dev/null; chown -R $(id -u):$(id -g) /out; exit \$status"
  docker rm -f ds_server >/dev/null 2>&1 || true
done
