#!/bin/bash
# Run LIGO on a rosbag inside Docker, then shut down cleanly so trajectories are written to ./output/Log.
#
# Usage:
#   docker/run_bag.sh <path/to/bag> [launch file] [config yaml] [rviz true|false]
# Example (M2DGR-plus Bridge_02):
#   docker/run_bag.sh ~/datasets/m2dgr_plus/bridge_02.bag mapping_robosense.launch m2dgr_bridge2.yaml
set -e

BAG="$(realpath "${1:?usage: $0 <bag> [launch] [config] [rviz]}")"
LAUNCH="${2:-mapping_robosense.launch}"
CONFIG="${3:-robosense.yaml}"
RVIZ="${4:-true}"
RATE="${RATE:-1.0}"

DATA_DIR="$(dirname "${BAG}")" exec "$(dirname "${BASH_SOURCE[0]}")/run.sh" bash -c "
    roscore > /dev/null 2>&1 &
    until rosparam list > /dev/null 2>&1; do sleep 0.5; done
    roslaunch ligo ${LAUNCH} config:=${CONFIG} rviz:=${RVIZ} &
    LAUNCH_PID=\$!
    sleep 5
    rosparam set use_sim_time false
    rosbag play -r ${RATE} '/data/$(basename "${BAG}")'
    sleep 5
    rosnode kill /laserMapping
    wait \${LAUNCH_PID}
"
