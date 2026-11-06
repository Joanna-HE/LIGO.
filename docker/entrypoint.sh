#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source "${CATKIN_WS:-/catkin_ws}/devel/setup.bash"
exec "$@"
