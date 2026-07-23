# Agribot 基于 MID360 FAST-LIO 的抗打滑里程计技术方案

## 1. 方案目标

本方案解决农田小车在以下情况下，轮速里程计仍错误累计位移的问题：

- 小车前方被障碍物顶住，但驱动轮继续旋转；
- 玉米地土壤松软，车轮发生完全或部分打滑；
- 轮速与车体真实运动不一致；
- 小车被外力推动，但轮速传感器没有检测到运动。

改进后的默认方案使用 MID360 点云和内置 IMU 运行 FAST-LIO，以激光惯性里程计作为车体真实运动的独立观测；轮速只在与 LIO 一致时参与高频预测。发生打滑后，系统立即停止使用轮速预测，并在持续不一致时完全隔离轮速。

系统同时保留原来的“轮速 + H30”里程计方案，可通过启动参数选择。无论选择哪种方案，对导航系统都保持以下接口不变：

```text
/odometry/filtered
odom -> base_footprint
```

中心线提取、PID 路径跟踪和 Pure Pursuit 路径跟踪不需要感知内部使用的是 FAST-LIO 还是原方案。

---

## 2. 原方案出现打滑误差的原因

底盘控制器返回的是轮子计算出的车体速度，而不是车体相对于地面的真实速度。

底盘节点收到串口速度后，将轮速积分为位置：

```cpp
robot_pos_.X +=
  (robot_vel_.X * cos(robot_pos_.Z) -
  robot_vel_.Y * sin(robot_pos_.Z)) * sample_time_;

robot_pos_.Y +=
  (robot_vel_.X * sin(robot_pos_.Z) +
  robot_vel_.Y * cos(robot_pos_.Z)) * sample_time_;

robot_pos_.Z += robot_vel_.Z * sample_time_;
```

对应代码：

```text
src/turn_on_32chassis/src/turn_on_32chassis.cpp
```

当车轮原地空转时：

```text
编码器检测到轮子旋转
        ↓
底盘计算得到非零速度
        ↓
/wheel/odom 发布非零速度
        ↓
EKF继续积分速度
        ↓
/odometry/filtered 错误移动
```

H30 可以提供较稳定的 yaw 和 yaw rate，但不能直接测量车体平移。因此，H30 可以约束航向，却无法判断“轮子转了但车体没有向前移动”。只使用轮速和 H30，无法从观测层面可靠区分正常行驶与完全空转。

此外，原轮速消息中的部分协方差设置得过小，并存在不合理的非对角项，会让 EKF 对轮速过度自信，加剧打滑后的错误累计。

---

## 3. 使用的核心技术

### 3.1 FAST-LIO 激光惯性里程计

FAST-LIO 使用 MID360 点云和内置 IMU，通过紧耦合迭代误差状态卡尔曼滤波估计：

- 三维位置；
- 三维姿态；
- 车体速度；
- IMU 零偏；
- 局部地图与当前点云之间的几何约束。

输入为：

```text
/livox/lidar
/livox/imu
```

原始输出为：

```text
/Odometry
```

FAST-LIO 不使用轮速，因此轮子空转时，只要车体没有相对于玉米、地面或其他环境结构移动，LIO 估计的平移就应接近零。这样便获得了一个独立于轮速的真实运动参考。

FAST-LIO 提供的是局部相对里程计，不是全局地图定位。它解决的是短期和中期真实运动观测问题，仍会随时间缓慢漂移。

### 3.2 robot_localization EKF

FAST-LIO 模式下，`robot_localization` EKF 融合：

- `/wheel/odom_validated`：通过打滑检查的轮速平移速度；
- `/lio/odom`：对齐到 `odom` 后的 LIO 平移位置和速度；
- `/imu/selected`：H30 的相对 yaw 和 yaw rate。

轮速不再提供最终航向，防止空转时轮速角速度驱动航向变化。H30 继续作为主要航向来源。

EKF 的输出是内部话题：

```text
/odometry/fused_internal
```

该 EKF 不发布 TF。最终里程计和 `odom -> base_footprint` TF 由抗打滑管理节点统一发布。

### 3.3 轮速一致性检测和故障隔离

新增的抗打滑管理节点不是简单地对轮速和 LIO 做平均，而是把 LIO 当作独立运动基准，对轮速进行完整性监测。

使用两级判定：

1. 瞬时速度残差；
2. 时间窗口内的位移和转角残差。

检测到不一致后，通过动态协方差、停止预测和完全隔离三个层级处理轮速。

### 3.4 有界短时航位推算

MID360 点云频率约为 10 Hz，而导航控制需要更高频率的连续里程计。最终输出以 50 Hz 发布：

- 每个 FAST-LIO 新帧到达时，直接采用最新 LIO 的 x/y；
- 两帧 LIO 之间，轮速可信时使用轮速做短时插值；
- 轮速不可用时使用最新 LIO 速度做最长 0.15 秒的短时预测；
- 所有预测都有时间和距离上限，不允许无限积分。

这种设计兼顾了 LIO 的真实性和轮速的高频实时性。

---

## 4. 系统数据流

### 4.1 FAST-LIO 抗打滑模式

```text
MID360 PointCloud2 ── /livox/lidar ─┐
                                    ├─ FAST-LIO ─ /Odometry
MID360 内置IMU ───── /livox/imu ────┘                 │
                                                      │
/wheel/odom ──────────────────────────────────────────┤
                                                      ▼
                                           slip_aware_odometry
                                             │             │
                                             │             └─ /lio/odom
                                             │
                                             └─ /wheel/odom_validated
                                                        │
H30 ─ /imu/data_h30 ─ imu_source_selector ─ /imu/selected
                                                        │
                                                        ▼
                                            robot_localization EKF
                                                        │
                                           /odometry/fused_internal
                                                        │
                                                        ▼
                                           slip_aware_odometry
                                                        │
                               ┌────────────────────────┴──────────────┐
                               ▼                                       ▼
                    /odometry/filtered                     odom -> base_footprint
```

### 4.2 原始 legacy 模式

```text
/wheel/odom ───────────────┐
                           ├─ robot_localization EKF
/imu/selected（默认H30）───┘              │
                                         ├─ /odometry/filtered
                                         └─ odom -> base_footprint
```

legacy 模式不启动 FAST-LIO 和抗打滑管理节点，因此不具备可靠的轮速打滑隔离能力。

MID360 驱动在两种模式下都会启动，继续为玉米行中心线提取和障碍物检测提供 `/livox/lidar`。

---

## 5. 打滑检测算法

## 5.1 输入数据有效性检查

抗打滑管理节点首先检查 FAST-LIO 数据：

- pose 和 twist 中不能出现 NaN 或 Inf；
- x/y 位置协方差必须有效；
- x/y 位置方差不得超过 `0.25 m²`；
- 时间戳必须严格递增；
- 处理延迟不能超过 `0.30 s`。

无效、乱序、严重退化或明显过期的 LIO 消息不会进入最终里程计。

## 5.2 坐标系对齐

FAST-LIO 输出的是：

```text
lio_odom -> lio_body
```

最终导航需要：

```text
odom -> base_footprint
```

管理节点先使用 MID360 惯导主体到 `base_footprint` 的外参，把 LIO 惯导主体位姿转换为车体位姿：

```text
T(lio, base) = T(lio, lio_body) × inverse(T(base, lio_body))
```

第一次收到有效 LIO 时，再计算：

```text
T(odom, lio) = T(odom, target) × inverse(T(lio, base))
```

这样 FAST-LIO 可以从自己的任意局部原点开始，但最终输出仍从当前 `odom` 原点或冻结位置继续，不会因为 FAST-LIO 重启造成 TF 突跳。

当前外参初值为：

```yaml
base_to_lio_body.x: 0.04263
base_to_lio_body.y: 0.02338
base_to_lio_body.z: 0.07088
base_to_lio_body.roll: 0.0
base_to_lio_body.pitch: 0.0
base_to_lio_body.yaw: 0.0
```

这些是 CAD 和 MID360 出厂外参组合得到的初值，实车验收前仍需标定。

## 5.3 瞬时速度残差

每次收到轮速消息时，立即比较最新轮速与最新 LIO 速度：

```text
e_v = ||v_wheel - v_lio||
e_w = |yaw_rate_wheel - yaw_rate_lio|
```

当前参数：

```yaml
instant_linear_residual_threshold: 0.15
instant_linear_recovery_threshold: 0.08
instant_yaw_rate_residual_threshold: 0.50
instant_yaw_rate_recovery_threshold: 0.25
minimum_observable_speed: 0.05
```

当最大运动速度低于 `0.05 m/s` 时，不进行瞬时速度判定，避免静止噪声误触发。

如果速度差超过门限：

- 立即停止轮速帧间预测；
- 轮速协方差至少放大 100 倍；
- 不等待完整位移比较窗口。

这一层主要解决“小车刚被障碍物顶住，轮子突然开始空转”的快速响应问题。

恢复门限小于触发门限，构成迟滞区，防止状态在阈值附近反复切换。

## 5.4 时间窗口位移残差

系统在时间上同步轮速和 LIO 位姿，当前允许最大时间差为 `0.15 s`。

在 `0.2 s` 比较窗口内计算相对运动：

```text
ΔT_wheel = inverse(T_wheel_start) × T_wheel_end
ΔT_lio   = inverse(T_lio_start)   × T_lio_end
```

平移残差：

```text
e_translation =
  sqrt((Δx_wheel - Δx_lio)² + (Δy_wheel - Δy_lio)²)
```

航向残差：

```text
e_yaw = abs(normalize(Δyaw_wheel - Δyaw_lio))
```

当前主要阈值：

```yaml
comparison_window: 0.2
translation_residual_threshold: 0.025
yaw_residual_threshold: 0.12
minimum_observable_motion: 0.01
slip_confirmation_time: 0.2
```

只有轮速或 LIO 的实际位移大于 `0.01 m` 时才判断，避免把纯静止噪声当作打滑。

## 5.5 轮速可信状态机

状态机包含四个状态：

```text
TRUSTED -> SUSPECT -> REJECTED -> RECOVERING -> TRUSTED
```

### TRUSTED

- 轮速与 LIO 一致；
- 轮速正常进入 EKF；
- 允许轮速用于 LIO 帧间高频预测；
- 协方差倍率为 1。

### SUSPECT

- 首次检测到明显位移或转角不一致；
- 立即停止轮速预测；
- 轮速仍可带高协方差进入 EKF；
- 协方差从可疑倍率逐渐增大；
- 当前可疑初始倍率为 25。

如果不一致在 `0.2 s` 内消失，则回到 `TRUSTED`；持续存在则进入 `REJECTED`。

### REJECTED

- 不再发布 `/wheel/odom_validated`；
- 轮速完全不参与最终位置预测；
- 近似隔离协方差倍率为 `1,000,000`；
- 最终平移跟随 LIO，不再跟随空转轮速。

### RECOVERING

轮速与 LIO 的残差必须同时满足：

```yaml
recovery_translation_threshold: 0.015
recovery_yaw_threshold: 0.06
```

连续一致 `0.5 s` 后进入恢复状态，再用 `0.5 s` 将轮速预测权重从 0 平滑增加到 1。

如果恢复期间再次不一致，立即返回 `REJECTED`。该设计避免轮速突然重新加入造成位置或速度跳变。

---

## 6. 最终里程计如何生成

最终 `/odometry/filtered` 不是直接采用 EKF 的全部 pose。

### 6.1 平移位置

x/y 主要来自低延迟 LIO 路径：

```text
最新LIO位置
  + 可信轮速的短时位移插值
```

轮速预测满足以下限制：

```yaml
maximum_prediction_horizon: 0.15
maximum_wheel_prediction_distance: 0.06
```

即使轮速出现异常，单次预测也不会无限扩张。

如果轮速处于可疑、隔离状态或瞬时不一致，则改为：

```text
最新LIO位置 + 最新LIO速度 × 最长0.15秒
```

每个新的 FAST-LIO 帧到达后，最新 LIO x/y 会立即成为最终位置，不再等待 EKF 数秒后慢慢拉回。

### 6.2 航向

最终 orientation 取自 EKF 的融合航向，主要由 H30 的相对 yaw 和 yaw rate 约束。

FAST-LIO 模式下轮速只融合 vx 和 vy，不融合轮速 yaw/yaw rate，因此车轮空转不会直接驱动最终航向。

### 6.3 速度

- 轮速可信时，可以使用 EKF 中的高频融合速度；
- 轮速不可信时，线速度切换为 LIO 速度；
- 静止保持或 LIO 超时冻结时，线速度清零。

---

## 7. 实时性改进

早期实现的问题是先让轮速持续积分，再等待较慢的 LIO/EKF 把位置拉回。表现为小车已经运动一至两秒，里程计才发生修正，不适合玉米地实时路径跟踪。

当前实现从以下方面降低延迟。

### 7.1 最终位置直接使用最新 LIO

管理节点每次收到 FAST-LIO 帧时，直接更新最终 x/y。EKF 只负责速度与航向融合，不再作为平移位置的延迟输出通道。

### 7.2 使用 FAST-LIO 当前 ESKF 速度

FAST-LIO 输出当前滤波状态中的车体系速度。管理节点优先使用该速度，而不是只通过相邻 pose 差分计算速度，避免再增加一个雷达扫描周期的相位延迟。

当前速度滤波系数：

```yaml
twist_filter_alpha: 0.8
```

系数越大越实时，越小越平滑。

### 7.3 只保留最新雷达帧

FAST-LIO 使用：

```yaml
lidar_qos_depth: 1
maximum_lidar_buffer_size: 1
imu_qos_depth: 400
```

算力不足时丢弃旧雷达帧，只处理最新帧，避免形成数秒的点云处理积压。IMU 队列保留 400 个样本，以便跨过被跳过的雷达帧继续惯性传播。

### 7.4 关闭非必要可视化输出

实机导航默认关闭：

- FAST-LIO Path；
- 注册点云；
- 地图点云；
- body-frame 点云；
- PCD 保存；
- FAST-LIO 自身 TF。

这些输出会增加点云复制、转换和 DDS 传输负担，关闭后将算力优先留给扫描匹配。

---

## 8. 静止抖动处理

FAST-LIO 在车体静止时仍可能产生毫米级点云匹配噪声。如果直接将这些位置发布为 TF，小车模型和路径跟踪会出现小幅抖动。

系统增加静止保持逻辑。

满足以下条件持续 `0.30 s`：

- LIO 平移速度不超过 `0.025 m/s`；
- 同时轮速静止，或者轮速与 LIO 明显不一致。

系统对静止候选期间的 x/y 求平均，然后锁定最终 x/y。

当前参数：

```yaml
stationary_hold_time: 0.30
stationary_lio_speed_threshold: 0.025
stationary_lio_release_speed: 0.05
stationary_wheel_linear_threshold: 0.01
stationary_wheel_angular_threshold: 0.02
stationary_release_distance: 0.02
```

出现以下任一情况时解除锁定：

- 可信轮速检测到真实运动；
- LIO 速度超过 `0.05 m/s`；
- LIO 真实位置相对锁定点移动超过 `0.02 m`。

锁定只作用于 x/y。yaw 继续由 H30 更新，避免强行锁死航向。

当小车被障碍物顶住且轮子空转时，LIO 速度低、轮速与 LIO 不一致，因此系统仍可进入静止保持，阻止轮速空转造成位置漂移。

---

## 9. LIO 失效保护

抗打滑方案的原则是：没有独立运动参考时，不允许退回到可能正在打滑的轮速。

### 9.1 LIO短时间未更新

LIO 超过 `0.15 s` 未更新后：

- 不再转发新的可信轮速；
- 不再使用轮速继续推算；
- 最多保留 0.15 秒的有界 LIO 速度预测。

### 9.2 LIO超时

LIO 超过 `0.5 s` 没有有效数据后：

- 冻结在最后可信位置；
- twist 置零；
- x/y/yaw 及速度协方差提高到 1000；
- 轮速状态切换为隔离；
- 清除旧的运动比较历史。

### 9.3 LIO恢复

恢复后的第一帧 LIO 会重新对齐到冻结位置：

```text
新的LIO局部坐标系原点
        ↓ 重新对齐
冻结的odom位置
```

即使 FAST-LIO 自身重启后坐标从很远的位置开始，最终 `odom -> base_footprint` 也不会突跳。轮速需要重新证明与 LIO 一致后才能逐渐恢复。

---

## 10. FAST-LIO 的工程改造

FAST-LIO 源码位于：

```text
src/FAST_LIO/
```

### 10.1 MID360输入配置

文件：

```text
src/FAST_LIO/config/mid360.yaml
```

主要改动：

- 点云话题统一为 `/livox/lidar`；
- IMU话题统一为 `/livox/imu`；
- PointCloud2 模式使用 `lidar_type: 4`；
- 使用 MID360 的 4 线扫描配置；
- 关闭软件自动时间同步；
- 使用标定的 LiDAR 到 IMU 外参；
- 实机关闭 PCD 保存和额外可视化输出；
- 配置实时 DDS 和处理队列。

### 10.2 正确发布速度和协方差

文件：

```text
src/FAST_LIO/src/laserMapping.cpp
```

原 FAST-LIO ROS 2 输出中补充了：

- 当前 ESKF 的 pose covariance；
- 世界系速度转换到车体系后的 twist；
- 车体系速度 covariance；
- 当前帧退化判定；
- 退化时将协方差放大 1000 倍。

退化判定使用：

```text
有效特征点数量 < 20
或
平均匹配残差 > 0.20
```

管理节点可根据这些协方差拒绝严重退化的 LIO 数据。

### 10.3 禁止FAST-LIO发布冲突TF

增加 `publish.tf_en` 参数并在实机配置中设为 `false`。

FAST-LIO 只发布 `/Odometry`，最终 TF 由抗打滑管理节点发布，避免多个节点同时发布 `odom -> base_footprint`。

### 10.4 实时队列

新增只保留最新点云的队列处理逻辑。新的点云到达时，如果处理缓冲区中仍有旧扫描，则丢弃旧扫描并选择最新帧，避免处理延迟持续增长。

---

## 11. robot_localization 配置改造

### 11.1 FAST-LIO模式

文件：

```text
src/turn_on_32chassis/config/ekf.yaml
```

配置为：

- 50 Hz 输出；
- 二维模式；
- `/wheel/odom_validated` 只融合 vx、vy；
- `/lio/odom` 融合 x、y、vx、vy；
- `/imu/selected` 融合相对 yaw 和 yaw rate；
- 不融合轮速航向；
- `publish_tf: false`；
- 内部输出重映射为 `/odometry/fused_internal`。

### 11.2 legacy模式

文件：

```text
src/turn_on_32chassis/config/ekf_legacy.yaml
```

保持原始方案：

- `/wheel/odom` 提供 vx、vy 和 yaw rate；
- `/imu/selected` 提供相对 yaw；
- EKF 直接发布 `/odometry/filtered`；
- EKF 直接发布 `odom -> base_footprint`；
- 不运行 FAST-LIO 抗打滑管理。

---

## 12. 原始轮速协方差修正

文件：

```text
src/turn_on_32chassis/include/turn_on_32chassis/turn_on_32chassis.hpp
```

主要改动：

- 移除异常的非对角协方差项；
- 避免使用 `1e-9` 这种几乎绝对可信的轮速方差；
- 增大运动状态下的 pose 和 twist 方差；
- 未观测的 z、roll、pitch 等维度保持大方差；
- 静止与运动状态使用不同但合理的对角协方差。

修正后的典型值：

```text
运动时轮速线速度方差：0.02
静止时轮速线速度方差：0.0025
运动时yaw rate方差：0.05
静止时yaw rate方差：0.01
```

FAST-LIO模式下，管理节点还会根据轮速可信状态进一步动态放大协方差。

---

## 13. H30接入

文件：

```text
src/turn_on_32chassis/src/imu_source_selector.cpp
```

IMU选择节点将 H30 或板载 IMU 统一输出为：

```text
/imu/selected
```

节点会：

- 检查四元数是否有效；
- 对四元数归一化；
- 输入未设置协方差时填写默认协方差；
- 向 EKF 提供统一格式的 `sensor_msgs/Imu`。

默认配置选择 H30：

```yaml
imu_source: h30
start_h30_driver: true
h30_topic: /imu/data_h30
h30_serial_port: /dev/wheeltec_IMU
h30_baud_rate: 460800
h30_frame_id: gyro_link
```

---

## 14. 启动链路和模式开关

主要启动文件：

```text
src/turn_on_32chassis/launch/turn_on_32chassis.launch.py
src/turn_on_agribot/launch/turn_on_agribot.launch.py
```

参数：

```yaml
odometry_mode: fastlio
```

可选值：

```text
fastlio
legacy
```

启动命令：

```bash
# 默认FAST-LIO抗打滑模式
ros2 launch turn_on_agribot turn_on_agribot.launch.py

# 显式选择FAST-LIO
ros2 launch turn_on_agribot turn_on_agribot.launch.py \
  odometry_mode:=fastlio

# 使用原轮速+H30方案
ros2 launch turn_on_agribot turn_on_agribot.launch.py \
  odometry_mode:=legacy
```

FAST-LIO模式只启动：

- FAST-LIO；
- slip-aware odometry；
- FAST-LIO融合EKF。

legacy模式只启动：

- legacy EKF。

两套最终里程计发布链路严格互斥，避免出现两个 `/odometry/filtered` 发布者或两个 `odom -> base_footprint` TF 发布者。

该开关是启动时参数，不支持运行中热切换。切换后需要重新启动 launch，以避免状态、时间戳和 TF 连续性问题。

顶层 `turn_on_agribot.launch.py` 只包含一次底盘总启动文件。MID360 驱动统一由 `turn_on_32chassis.launch.py` 启动，消除了原来上下两层 launch 重复启动雷达驱动的问题。

---

## 15. 点云消费者统一

以下代码原来订阅 `/mid360_PointCloud2`，现改为参数化订阅，默认使用：

```text
/livox/lidar
```

涉及文件：

```text
src/centerline_extraction/src/corn_row_detector_projection.cpp
src/centerline_extraction/src/obstacle_detector.cpp
```

新增参数：

```text
point_cloud_topic
```

这样 FAST-LIO、玉米行中心线提取和障碍物检测共用一个 MID360 驱动输出，不需要重复启动驱动或复制点云话题。

---

## 16. 导航接口

无论采用哪一种里程计方案，中心线提取和路径跟踪都使用：

```text
/odometry/filtered
```

当前以下控制器已经订阅该话题：

```text
src/centerline_extraction/src/pure_pursuit_controller.cpp
src/centerline_extraction/src/pid_controller.cpp
```

不要让导航控制器直接使用：

```text
/wheel/odom
/Odometry
/lio/odom
/odometry/fused_internal
```

这些都是原始或内部话题。

---

## 17. 主要参数汇总

参数文件：

```text
src/turn_on_32chassis/config/bringup.yaml
```

| 类别 | 参数 | 当前值 | 作用 |
|---|---:|---:|---|
| 模式 | `odometry_mode` | `fastlio` | 选择抗打滑或原方案 |
| 比较 | `comparison_window` | 0.2 s | 位移残差比较窗口 |
| 同步 | `maximum_sync_delta` | 0.15 s | 轮速与LIO最大时间差 |
| 新鲜度 | `wheel_lio_max_age` | 0.15 s | 超过后停止使用轮速 |
| 超时 | `lio_timeout` | 0.5 s | 超过后冻结 |
| 输出 | `output_frequency` | 50 Hz | 最终里程计频率 |
| 预测 | `maximum_prediction_horizon` | 0.15 s | 最大短时预测时间 |
| 预测 | `maximum_wheel_prediction_distance` | 0.06 m | 单次轮速预测上限 |
| 延迟 | `maximum_lio_latency` | 0.30 s | 拒绝积压的LIO |
| 瞬时残差 | `instant_linear_residual_threshold` | 0.15 m/s | 立即停止轮速预测 |
| 位移残差 | `translation_residual_threshold` | 0.025 m | 进入可疑状态 |
| 航向残差 | `yaw_residual_threshold` | 0.12 rad | 进入可疑状态 |
| 确认 | `slip_confirmation_time` | 0.2 s | 持续后完全隔离 |
| 恢复 | `wheel_recovery_time` | 0.5 s | 一致状态确认 |
| 恢复 | `wheel_recovery_ramp_time` | 0.5 s | 权重渐进恢复 |
| 静止 | `stationary_hold_time` | 0.30 s | 静止锁定确认 |
| 静止 | `stationary_release_distance` | 0.02 m | 外力推动释放门限 |
| 冻结 | `frozen_variance` | 1000 | LIO失效协方差 |

这些参数是当前工程初值，应使用同一组 rosbag 反复回放进行调优。

---

## 18. 测试

### 18.1 状态机单元测试

文件：

```text
src/turn_on_32chassis/test/test_wheel_trust_monitor.cpp
```

覆盖：

- 正常轮速持续可信；
- 完全打滑；
- 部分平移打滑；
- 转弯航向打滑；
- 轮速为零但车体被推动；
- 完全静止时忽略噪声；
- 持续一致后恢复；
- 启动时没有 LIO 不信任轮速。

### 18.2 ROS合成消息测试

文件：

```text
src/turn_on_32chassis/test/test_slip_aware_odometry_launch.py
```

覆盖：

- 正常运动；
- 新 LIO 位置立即进入最终输出；
- 静止 LIO 噪声抑制；
- 轮速瞬时空转立即停止预测；
- 持续打滑后停止发布可信轮速；
- LIO 乱序；
- LIO NaN；
- LIO超时冻结；
- 冻结后速度清零；
- 冻结后协方差升高；
- LIO恢复时重新对齐且不跳变。

当前包级测试结果：

```text
37 tests
0 errors
0 failures
8 skipped
```

已经完成 `colcon build`、包级测试和 launch 参数检查。

---

## 19. 需要继续进行的实车工作

源码和合成测试不能替代以下实车验收。

### 19.1 外参标定

必须实测：

- `base_link -> laser_link`；
- `base_link -> gyro_link`；
- `base_footprint -> MID360 IMU body`；
- MID360 LiDAR 到内置 IMU 外参。

外参错误会让转弯运动被错误解释为平移残差，并可能造成误判。

### 19.2 时间同步

需要检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic delay /Odometry
ros2 topic echo /livox/lidar --field header.stamp
ros2 topic echo /livox/imu --field header.stamp
```

如果已经完成硬件同步，不应打开 FAST-LIO 软件时间同步。存在稳定固定偏差时，应标定 `time_offset_lidar_to_imu`。

### 19.3 rosbag场景

建议同时录制：

```bash
ros2 bag record \
  /livox/lidar /livox/imu /imu/data_h30 /imu/selected \
  /wheel/odom /wheel/odom_validated /Odometry /lio/odom \
  /odometry/fused_internal /odometry/filtered \
  /cmd_vel /tf /tf_static
```

至少录制：

- 静止；
- 正常直行；
- 正常转弯；
- 完全顶住后空转；
- 泥土中的部分打滑；
- 小车被外力推动；
- LIO遮挡或退化；
- 10分钟连续玉米行自主行驶。

### 19.4 验收指标

- 从静止开始完全顶住空转 10 秒，最终平移偏移不超过 2 cm；
- 部分打滑时，最终位移跟随 LIO 真实运动；
- 轮速恢复一致后平滑重新参与；
- LIO超时后位姿冻结、速度清零、协方差升高；
- LIO恢复后 TF 不突跳；
- 正常连续运行 10 分钟不误隔离轮速；
- 最终 `/odometry/filtered` 只有一个发布者；
- `odom -> base_footprint` 只有一个发布者；
- MID360 驱动只有一个实例。

---

## 20. 方案边界

1. 抗打滑能力依赖 FAST-LIO 有效。环境几何严重退化、点云被遮挡或时间同步异常时，系统会冻结，而不是冒险使用轮速。
2. 玉米行具有一定重复结构，仍需要实测纵向和横向可观性。H30 可以稳定航向，但不能替代 LIO 的平移观测。
3. 当前 2 cm 指标尚需实车 rosbag 和标定数据验证，不能只依靠单元测试确认。
4. legacy 模式保留兼容性，但不具备可靠的完全打滑检测能力。
5. 本方案只修正里程计，不发布额外打滑报警，不修改上层控制，也不自动停车。

---

## 21. 主要改动文件索引

| 文件 | 改动 |
|---|---|
| `src/FAST_LIO/config/mid360.yaml` | MID360话题、类型、外参、实时队列、关闭可视化与PCD |
| `src/FAST_LIO/src/laserMapping.cpp` | 最新帧队列、速度、协方差、退化标记、可选TF |
| `src/turn_on_32chassis/src/slip_aware_odometry.cpp` | 抗打滑管理、坐标对齐、预测、冻结、静止保持、最终odom和TF |
| `src/turn_on_32chassis/include/turn_on_32chassis/wheel_trust_monitor.hpp` | 四状态轮速可信状态机 |
| `src/turn_on_32chassis/config/bringup.yaml` | 模式和全部抗打滑参数 |
| `src/turn_on_32chassis/config/ekf.yaml` | FAST-LIO模式内部融合配置 |
| `src/turn_on_32chassis/config/ekf_legacy.yaml` | 原轮速+H30模式配置 |
| `src/turn_on_32chassis/launch/turn_on_32chassis.launch.py` | 两种模式互斥启动、统一驱动和TF发布者 |
| `src/turn_on_agribot/launch/turn_on_agribot.launch.py` | 只包含一次完整底盘启动链路 |
| `src/turn_on_32chassis/include/turn_on_32chassis/turn_on_32chassis.hpp` | 修正原始轮速协方差 |
| `src/turn_on_32chassis/src/imu_source_selector.cpp` | H30/板载IMU统一与协方差补全 |
| `src/centerline_extraction/src/corn_row_detector_projection.cpp` | 点云话题统一并参数化 |
| `src/centerline_extraction/src/obstacle_detector.cpp` | 点云话题统一并参数化 |
| `src/turn_on_32chassis/test/test_wheel_trust_monitor.cpp` | 状态机单元测试 |
| `src/turn_on_32chassis/test/test_slip_aware_odometry_launch.py` | 合成ROS消息集成测试 |

