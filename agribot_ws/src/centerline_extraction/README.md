# PID 室内固定轨迹测试

真实玉米田的感知质量约束导航请先阅读
[`docs/field_experiment_protocol.md`](docs/field_experiment_protocol.md)，并使用
两个独立启动：`field_system_with_perception.launch.py` 启动基础系统和中心线感知，
`field_path_tracking.launch.py` 启动路径跟踪和本次试验记录。田间正常运行只使用
这两个入口。该流程会保存全部质量分量、参数快照和安全停车状态；不要用本页的
室内固定轨迹配置直接下田。

本测试使用 3 × 6 块、每块 0.60 m 的地面，即 **1.8 m × 3.6 m** 的区域。
默认轨迹以小车收到第一帧 `/odometry/filtered` 时的位姿为起点，因此不需要把
odom 数值清零。将车放在区域短边中央，车头对准 3.6 m 长边；车的实际外廓应在
起点前、后和两侧保留安全余量。

默认 `straight_half_circle` 为：

```text
起点 ── 1.50 m 直线 ── 左转 180°半圆（R=0.25 m）── 1.00 m 反向直线 ── 终点
```

总弧长约 3.29 m。若从场地宽度中线出发，半圆后的终点横向偏移 0.50 m；机器人
最大宽度为 0.25 m 时，仍应检查车体外廓与场地边界的实际安全间隙。
到达终点 0.12 m 范围内时，
PID 控制器会发布零速度；若因横向误差越过终点横线而未进入该圆形范围，也会立即
发布零速度。该停车行为只在本测试配置中开启。

## 启动

先启动底盘和里程计：

```bash
ros2 launch turn_on_agribot turn_on_agribot.launch.py
```

确认车轮悬空或急停可用后，在另一终端执行：

```bash
cd /home/wheeltec/agribot/agribot_ws
colcon build --packages-select centerline_extraction --symlink-install
source install/setup.bash
ros2 launch centerline_extraction pid_path_test.launch.py
```

默认最大线速度为 **0.12 m/s**。由于圆弧半径为 0.25 m，建议不高于 **0.15 m/s**；
如需临时修改，使用 launch 参数（而不是直接传给
ROS 2 launch 命令的未声明参数）：

```bash
ros2 launch centerline_extraction pid_path_test.launch.py max_linear_speed:=0.25
```

测试路径会在收到第一帧 `/odometry/filtered` 后固定。测试时不要启动
`corn_row_detector_projection`，否则会有两个节点同时发布
`/corn_row_center_line`。

测试结束、控制器停车后，`pid_tracking_evaluator` 会自动结束并在
工作空间的 `pid_tracking_results/run_日期_时间/` 生成：

- `tracking_samples.csv`：每个采样时刻的位姿、横向误差、航向误差及控制指令；
- `report.json`：RMSE、P95、最大误差、终点误差、转向饱和率与振荡统计；
- `recommended_pid.yaml`：根据本轮结果生成的下一轮候选参数。

候选参数不会自动应用。必须停车后检查报告，再将其中数值人工写入
`config/pid_path_test.yaml` 后重新编译/重启测试。评估器每轮只建议一个调参方向，
避免同时改变多个变量而无法判断效果。

可用下面命令看路径、跟踪目标和控制输出：

```bash
ros2 topic echo /corn_row_center_line --once
ros2 topic echo /target_point
ros2 topic echo /cmd_vel
```

## 建议调参顺序

配置文件是 `config/pid_path_test.yaml`。初值只使用 P 项，速度限制为 0.18 m/s：

1. 将 `profile` 改为 `straight`，先调 `heading_kp`：小幅增加直至方向修正变快但不来回摆动；若方向相反，先停止测试，检查 odom yaw 和车体正方向，不要靠负增益补偿。
2. 改回 `straight_half_circle`，先确认进入圆弧前的直线段稳定，再观察 180°半圆是否出现角速度饱和或越界。
3. 圆弧持续出现滞后或超调时，先降低速度；D 项一旦使角速度噪声或高频抖动变大就回退。
4. 最后才尝试积分项：仅当直线段存在稳定的单侧偏差时，将对应 `*_ki` 从 0.01 开始。每次只改一个参数，并记录最大横向偏差和是否振荡。

调参过程请始终限制 `max_linear_speed`，确认低速稳定后再以 0.02–0.03 m/s 的步长增加。若发现异常，立即急停并终止测试节点；不要在窄场地中直接提高角速度或速度上限。
