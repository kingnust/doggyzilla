#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /root/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"

exec "$@"
