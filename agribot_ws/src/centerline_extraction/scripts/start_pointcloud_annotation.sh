#!/usr/bin/env bash

set -e

if [ "$#" -lt 1 ]; then
  echo "用法：$0 ROSBAG目录 [其他标注器参数]"
  exit 2
fi

ANNOTATION_BAG_PATH="$1"
shift

source /opt/ros/humble/setup.bash
source /home/wheeltec/agribot/agribot_ws/install/setup.bash

exec ros2 run centerline_extraction pointcloud_centerline_annotator.py \
  --bag "$ANNOTATION_BAG_PATH" "$@"
