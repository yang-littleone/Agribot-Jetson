# Agribot 抗打滑里程计

## 数据链路

实机启动入口仍为：

```bash
ros2 launch turn_on_agribot turn_on_agribot.launch.py
```

`config/bringup.yaml` 的 `launch.odometry_mode` 用于选择里程计：

- `fastlio`（默认）：启用FAST-LIO和抗打滑管理节点；
- `legacy`：使用原来的轮速+H30方案，EKF直接发布最终odom和TF。

也可以在命令行临时切换：

```bash
ros2 launch turn_on_agribot turn_on_agribot.launch.py odometry_mode:=fastlio
ros2 launch turn_on_agribot turn_on_agribot.launch.py odometry_mode:=legacy
```

两种模式互斥，都会保持 `/odometry/filtered` 和
`odom -> base_footprint` 接口不变。`legacy` 不具备LIO抗打滑约束，轮子
空转时仍可能累计虚假里程。MID360驱动在两种模式下都启动，继续为玉米行
检测和障碍物检测提供 `/livox/lidar`；只有 `fastlio` 模式启动FAST-LIO
计算。

FAST-LIO模式下，启动链路只创建一个 MID360 驱动。FAST-LIO 使用
`/livox/lidar` 与 `/livox/imu`，输出原始 `/Odometry`。抗打滑管理节点
把 LIO 位姿转换到 `base_footprint`，输出 `/lio/odom`，并根据轮速与
LIO 的窗口位移残差决定是否把 `/wheel/odom` 转发为
`/wheel/odom_validated`。

最终里程计不再等待 EKF 缓慢修正平移。每个 FAST-LIO 帧到达时，其
对齐后的 x/y 会立即写入 `/odometry/filtered`；10 Hz 雷达帧之间由
轮速做最高 0.15 秒、最多 6 厘米的高频插值。每个轮速消息都会与最新
LIO 速度立即比较；检测到瞬时速度不一致或
窗口位移不一致后，轮速插值立即停用，下一帧 LIO 直接给出真实位置，
不会继续累计轮速误差。

小车静止时，FAST-LIO 自身约数毫米的点云匹配噪声不会直接传到 TF。
轮速和 LIO 速度连续 0.3 秒满足静止条件后，管理节点锁定最终 x/y 并将
平移速度置零。轮速恢复、LIO 速度超过门限，或外力推动造成 2 厘米以上
真实位移时会立即解除；yaw 仍持续取自 H30，不进行锁死。

`robot_localization` 只负责内部速度与航向融合：

- `/wheel/odom_validated`：车体平面平移速度，不使用轮速 yaw；
- `/lio/odom`：平移位置和速度；
- `/imu/selected`：H30 相对 yaw 和 yaw rate。

EKF 的内部输出为 `/odometry/fused_internal`，不发布 TF。只有
`slip_aware_odometry_node` 发布最终 `/odometry/filtered` 和
`odom -> base_footprint`；最终 x/y 取自低延迟 LIO 路径，yaw 取自
H30 融合结果。

当 LIO 超过 0.5 秒没有有效数据时，最终里程计冻结、速度清零并提高
协方差。LIO 恢复后以冻结位姿为新对齐点继续输出。系统不会
在 LIO 无效时退回未经验证的轮速。

实机默认关闭 FAST-LIO 的 Path、地图云和注册点云发布，避免
可视化负载拖慢扫描匹配。RViz由 `launch.use_rviz` 控制（当前默认开启）。
玉米行检测和障碍物检测仍直接订阅
`/livox/lidar`。FAST-LIO 的雷达 DDS 队列和内部处理队列都只保留
最新帧；算力短时不足时允许跳过旧雷达帧，同时保留最多 400 个 IMU
样本继续传播，避免逐帧处理旧队列形成秒级时延。需要现场调试时可执行：

```bash
ros2 launch turn_on_agribot turn_on_agribot.launch.py use_rviz:=true
```

## 实机标定

当前 `config/bringup.yaml` 中的 `base_to_lio_body.*` 是由现有 CAD
`base_link -> laser_link` 和 FAST-LIO 的 MID360 LiDAR/IMU 出厂外参
组合得到的初值：

```text
T(base, MID360_IMU) =
T(base, laser_link) * inverse(T(MID360_IMU, LiDAR))
```

投入验收前必须实测并更新：

- `base_link -> laser_link` 的 xyz/rpy；
- `base_link -> gyro_link` 的 xyz/rpy；
- MID360 点云和内置 IMU 的硬件时间戳偏差。

静止采集时可先检查时间戳是否单调、频率是否稳定：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic delay /Odometry
ros2 topic echo /livox/lidar --field header.stamp
ros2 topic echo /livox/imu --field header.stamp
```

若已通过硬件同步，不要打开 FAST-LIO 的软件时间同步参数；若测得固定
偏差，应填写 `common.time_offset_lidar_to_imu`，不要靠放宽打滑阈值
掩盖时延。`/Odometry` 延迟超过 0.3 秒时，管理节点会拒绝该旧数据并
进入冻结保护，不能让旧位置重新参与自主控制。

## rosbag 回放与验收

建议每次同时录制下列原始与中间话题：

```bash
ros2 bag record \
  /livox/lidar /livox/imu /imu/data_h30 /imu/selected \
  /wheel/odom /wheel/odom_validated /Odometry /lio/odom \
  /odometry/fused_internal /odometry/filtered /cmd_vel /tf /tf_static
```

分别录制静止、正常直行/转弯、完全顶住空转、部分打滑和被外力推动。
调参时使用同一组 bag 重放，以免把环境变化误认为参数改善。重点参数在
`config/bringup.yaml` 的 `slip_aware_odometry.ros__parameters` 下：

- `comparison_window`、`translation_residual_threshold`、
  `yaw_residual_threshold`；
- `slip_confirmation_time`；
- `wheel_recovery_time` 与 `wheel_recovery_ramp_time`；
- `maximum_prediction_horizon`、`maximum_wheel_prediction_distance`；
- `maximum_lio_latency`、`lio_timeout` 和 LIO 协方差门限；
- 正常、可疑、隔离状态的轮速协方差倍率。

静态检查可确认最终接口只有一个发布者：

```bash
ros2 topic info /odometry/filtered --verbose
ros2 topic info /tf --verbose
ros2 node list
```

2 厘米空转指标和 10 分钟无误隔离指标必须在外参、时间同步完成后用
实车 bag 验证；源码单元测试不能替代这两项实测。
