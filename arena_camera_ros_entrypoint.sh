#!/bin/bash

set -e
cd /arena_camera_ros2/ros2_ws 
#rosdep fix-permissions
#rosdep update
rosdep install --from-paths src --ignore-src -r -y;
colcon build --symlink-install
source install/local_setup.bash

exec "$@"
