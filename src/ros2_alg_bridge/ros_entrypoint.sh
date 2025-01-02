#!/bin/bash
set -e

# setup ros2 environment
source "/home/ros2_alg_bridge/install/setup.bash" --
exec "$@"
