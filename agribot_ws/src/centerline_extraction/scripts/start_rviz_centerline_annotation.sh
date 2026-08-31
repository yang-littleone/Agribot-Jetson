#!/usr/bin/env bash

set -e

if [ "$#" -lt 1 ]; then
  echo "用法：$0 ROSBAG目录 [其他参数]"
  exit 2
fi

RVIZ_ANNOTATION_BAG_PATH="$1"
shift

# 默认使用独立ROS域，避免实车节点发布的同名odom/base_link TF干扰录包位姿。
export ROS_DOMAIN_ID="${RVIZ_ANNOTATION_DOMAIN_ID:-77}"

source /opt/ros/humble/setup.bash
source /home/wheeltec/agribot/agribot_ws/install/setup.bash

exec ros2 run centerline_extraction rviz_centerline_annotator.py \
  --bag "$RVIZ_ANNOTATION_BAG_PATH" "$@"
