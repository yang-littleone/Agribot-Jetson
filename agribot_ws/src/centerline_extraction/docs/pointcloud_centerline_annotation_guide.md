# 玉米行点云中心线快速真值标注工具

> 真实玉米地人工真值标注现已改用RViz原生显示和`Publish Point`点击，
> 请优先参阅
> [`rviz_centerline_annotation_guide.md`](rviz_centerline_annotation_guide.md)。
> 本文所述Matplotlib界面仅保留用于数据检查。

## 1. 工具解决什么问题

本工具直接读取录制的`rosbag`，将`/livox/lidar`点云依据包内
`/tf_static`自动变换到`base_link`，并显示为俯视图。标注者只需点击
左右两侧可确认的茎秆中心，工具自动拟合平行双行、计算真实中心线、
行距和航向，不再需要逐帧导出PCD、手工写坐标或手算直线。

原始rosbag始终只读。对于`.db3.zstd`压缩包，程序只在工作空间的
`field_ground_truth/pointcloud_annotation_cache/`中建立解压缓存，
不会解压或修改U盘上的原始文件。

工具刻意不显示当前中心线算法的输出，避免人工真值受到待评价算法的
影响。输出可用于计算“提取中心线相对独立人工真值”的MAE、RMSE、
P95和航向误差。

## 2. 编译

```bash
cd /home/wheeltec/agribot/agribot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select centerline_extraction
source install/setup.bash
```

## 3. 首次检查rosbag

先用`--inspect`确认包中有点云，并且能找到
`laser_link`到`base_link`的TF：

```bash
ros2 run centerline_extraction pointcloud_centerline_annotator.py \
  --bag "/media/wheeltec/KINGSTON/agribot_field_data/field_trial_bags/包目录" \
  --inspect
```

正常时会显示点云帧数、原始点数、ROI内点数和
`transform_source: bag_tf_static`。若显示“没有`/livox/lidar`点云
消息”，该包不能用于点云标注。

## 4. 启动标注

运动包默认利用`/odometry/filtered`仅筛选车辆实际运动的片段，并且每
行驶0.5 m抽取一帧。里程计只用于减少重复帧，不参与真值计算；因此
里程计的小幅漂移不会被当作中心线真值：

```bash
/home/wheeltec/agribot/agribot_ws/src/centerline_extraction/scripts/\
start_pointcloud_annotation.sh \
  "/media/wheeltec/KINGSTON/agribot_field_data/field_trial_bags/包目录" \
  --sample-distance 0.5 \
  --annotator "标注人姓名"
```

如果包内没有有效运动，工具默认只打开时间中部的代表帧。静止包只需
选取少量有代表性的帧，不应把同一静止位置的数百帧当成数百个独立
样本。可以在“帧导航”中直接输入帧号跳转。需要完全按时间抽样时使用
`--sample-distance 0 --sample-period 2.0`。

## 5. 每帧如何标

界面会根据当前显示器自动最大化，右侧控制区可以上下滚动。先使用
“三维上下文”页判断茎秆、叶片及玉米行的空间结构；鼠标左键拖动可
旋转，滚轮可缩放。二维和三维均保留原始点云的`intensity`字段，
使用与现场RViz一致的黑色背景、强度着色和不透明方形点。三维图默认
显示`z=-0.20～2.50 m`的完整近场高度，其中二维标注高度ROI使用更大
的方形点突出。默认点数上限为50000，当前约2万点/帧的实测包不会
发生显示抽稀；图标题会明确显示实际点数及“完整/显示抽稀”状态。

仍然难以判断时，点击“播放当前帧前后点云”或按`Space`，默认观察
锚定帧前后各3帧的短时变化。发现清晰帧后按`Space`暂停，然后点击
“锁定当前预览帧用于标注”。软件会返回二维标注页，并同时记录原始
锚定帧、实际标注帧、帧差和时间差。点击“取消并返回锚定帧”则放弃
这次动态选择。

1. 只点击能够确认属于当前车辆所在通道的左右两条内侧植株行。
2. 回到“标注俯视图”，点击“左行（y>0）”，依次点击左侧3个或更多
   茎秆中心。
3. 点击“右行（y<0）”，选择右侧3个或更多茎秆中心。
4. 检查红、蓝边界线是否穿过人工选择的植株行，黄色中心线是否位于
   两行中间，行距是否接近现场实测值。
5. 无歧义时点击“保存有效并下一帧”；某侧被叶片完全遮挡、缺株导致
   无法判断，或多行归属无法确定时，选择原因后点击
   “不可判定并下一帧”。不可判定帧必须保留，不能只留下容易帧。

点击默认会吸附到半径0.04 m内点云的局部中值。吸附错误时可取消
“吸附”后直接点击。快捷键为：

- `L`：左行；`R`：右行；
- `U`：撤销当前侧最后一点；`C`：清空本帧；
- `S`：保存有效；`I`：保存为不可判定；
- `N`：下一个抽样帧；`P`：上一个抽样帧。
- `Space`：播放或停止当前帧前后的短时点云。

## 6. 数据保存位置

默认输出到：

```text
/home/wheeltec/agribot/agribot_ws/field_ground_truth/
└── pointcloud_annotations/
    └── rosbag名称/
        ├── centerline_annotations.json
        └── centerline_ground_truth.csv
```

每次保存都会同时更新JSON和CSV，可以中途关闭后继续。CSV中的关键列：

- `cloud_stamp_ns`：与算法输出匹配所用的点云时间戳；
- `sampling_anchor_frame_index`：固定抽样得到的原始锚定帧；
- `frame_index`：动态观察后实际锁定并标注的帧；
- `selected_frame_delta`、`selected_time_delta_s`：实际标注帧相对
  锚定帧的帧差和时间差；
- `true_center_offset_m`：`base_link`原点处真实中心线横向位置，左正右负；
- `true_row_yaw_deg`：真实玉米行相对车头的航向角；
- `row_width_m`：人工双行的拟合行距；
- `status`：`valid`或`invalid`；
- `left_points_xy`、`right_points_xy`：人工点击的原始依据，可复核；
- `fit_rmse_m`：人工点击点对平行双行模型的拟合残差。

## 7. 论文标注建议

- 包3静态场景用于工具和算法调试，只能算一个位置，不能用于扩大独立
  样本量。
- 正式精度评价从未用于调参的运动包中按固定时间间隔抽样；正常、
  天然叶片遮挡、缺株和杂草场景都要保留。
- 推荐先标30帧做双人或同一人间隔一天的复核。若中心偏移差异较大，
  先统一“茎秆中心”和“最内侧行”的判定规则，再继续批量标注。
- 报告有效帧误差的同时，必须报告不可判定帧比例。不要删除难以识别
  的帧后只计算容易帧MAE。
- 算法参数锁定后，才在保留包上计算最终指标，避免同一批帧既调参又
  报告性能。

## 8. 常见问题

若包内缺少TF，但已准确测得雷达相对`base_link`的安装外参，可显式
指定（以下数值仅为已有包的示例，不应盲目用于新安装）：

```bash
ros2 run centerline_extraction pointcloud_centerline_annotator.py \
  --bag "包目录" \
  --manual-translation 0.031631 0.000091 0.115 \
  --manual-rpy-deg 0 0 0
```

若显示过慢，可降低`--max-plot-points`；它只影响画面抽稀，不改变
点击吸附使用的完整ROI点云，也不改变保存的真值。
