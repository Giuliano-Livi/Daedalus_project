#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /deps_ws/install/setup.bash
[ -f /ov_ws/install/setup.bash ] && source /ov_ws/install/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
exec "$@"