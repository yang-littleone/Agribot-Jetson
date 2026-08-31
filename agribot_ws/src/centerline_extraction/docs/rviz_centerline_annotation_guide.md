# RViz玉米行中心线真值标注

## 启动

```bash
/home/wheeltec/agribot/agribot_ws/src/centerline_extraction/scripts/\
start_rviz_centerline_annotation.sh \
  "/media/wheeltec/KINGSTON/agribot_field_data/field_trial_bags/包目录" \
  --sample-distance 0.5 \
  --annotator "标注人姓名"
```

脚本会同时打开RViz和一个窄条控制器。RViz显示冻结的
`/annotation/cloud`，内容来自原始`/livox/lidar`，保留全部点和
`intensity`，不需要回放rosbag，也不会修改原始包。

脚本默认设置`ROS_DOMAIN_ID=77`，使标注RViz与当前实车ROS节点隔离，
防止实车发布的同名`odom`、`base_link`变换混入录包位姿。需要更换
独立域时，可在启动前设置`RVIZ_ANNOTATION_DOMAIN_ID`。

## 标注

1. 在RViz工具栏选择一次`Publish Point`；工具会保持选中，可以连续
   点击多个点。
2. 调整视角，直接点击当前通道左、右两侧能够确认的茎秆点。
3. 点击点的`y>0`时自动记为左行红点，`y<0`时自动记为右行蓝点。
4. 左右至少各2点可以拟合，正式标注建议各3点以上。
5. 红、蓝边界线和黄色中心线会直接显示在同一个RViz场景中。
6. 点击控制器的“保存有效并进入下一帧”。无法判断时选择原因后保存
   为不可判定，不要删除困难帧。

点错可点击“撤销最后一点”或“清空本帧”。“上一/下一原始帧”用于在
锚定帧附近逐帧检查；动态播放找到清晰帧后暂停并锁定，软件会保存
锚定帧、实际标注帧、帧差和时间差。

## 坐标与结果

RViz固定坐标系必须保持为`odom`，配置中会同时显示：

- `OdomAxes`：录包试验的局部里程计参考坐标；
- `BaseLinkAxes`：当前所选点云帧的小车位置和车头方向；
- `RecordedTF`：二者之间的`odom → base_link`箭头、坐标轴、名称和
  TF树。

坐标轴颜色遵循ROS约定：红色为X轴、绿色为Y轴、蓝色为Z轴。标注节点
从当前点云时间附近的`/odometry/filtered`读取车体位姿，并结合录包中
的静态TF得到`odom → base_link`。控制器会显示两者的x、y位移以及
点云与里程计消息的匹配时间差。

冻结点云、人工点击点、边界线和中心线原始数据仍统一位于
`base_link`。当RViz在`odom`下显示时，由上述录包TF完成变换；使用
`Publish Point`点击后，软件会自动把`odom`坐标逆变换回`base_link`，
不需要人工换算。

结果仍保存到：

```text
/home/wheeltec/agribot/agribot_ws/field_ground_truth/
└── pointcloud_annotations/
    └── rosbag名称/
        ├── centerline_annotations.json
        └── centerline_ground_truth.csv
```

旧的Matplotlib标注界面保留用于数据检查，但正式人工真值标注优先使用
本RViz工具。
