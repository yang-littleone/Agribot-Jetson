# Pure Pursuit 控制器使用说明

## 概述

Pure Pursuit（纯追踪）算法是一种几何路径跟踪算法，通过寻找路径上距离当前位置固定前视距离的目标点，计算圆弧轨迹来生成角速度控制。

## 算法原理

1. **前视点查找**：在参考路径上找到距离机器人当前位置为 `lookahead_distance` 的点
2. **曲率计算**：计算机器人到前视点的圆弧曲率 κ = 2y/L²
   - y：前视点在机器人坐标系下的横向坐标
   - L：机器人在前视距离
3. **控制律**：角速度 ω = v × κ
   - v：线速度
   - κ：路径曲率

## 启动方式

### 方法 1：使用 launch 文件
```bash
ros2 launch centerline_extraction pure_pursuit.launch.py
```

### 方法 2：手动运行节点
```bash
ros2 run centerline_extraction pure_pursuit_controller
```

## 参数配置

### 核心参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `lookahead_distance` | double | 0.5 | 前视距离（米） |
| `min_lookahead_distance` | double | 0.3 | 最小前视距离 |
| `max_lookahead_distance` | double | 1.0 | 最大前视距离 |
| `linear_velocity` | double | 0.3 | 期望线速度（m/s） |
| `max_linear_velocity` | double | 0.5 | 最大线速度 |
| `min_linear_velocity` | double | 0.1 | 最小线速度 |
| `max_angular_velocity` | double | 1.0 | 最大角速度（rad/s） |
| `velocity_gain` | double | 0.5 | 速度增益（用于自适应调速） |
| `adaptive_lookahead` | bool | true | 是否启用自适应前视距离 |
| `debug_mode` | bool | true | 是否发布可视化标记 |

### 参数调优建议

#### 1. 前视距离 (`lookahead_distance`)
- **较小值 (0.3-0.5m)**：跟踪精度高，但可能导致震荡
- **较大值 (0.6-1.0m)**：运行更平滑，但响应延迟增加
- **推荐**：从 0.5m 开始，根据实际效果调整

#### 2. 线速度 (`linear_velocity`)
- **田间作业**：0.2-0.4 m/s
- **道路行驶**：0.5-1.0 m/s
- **注意**：速度越快，需要越大的前视距离

#### 3. 自适应前视距离 (`adaptive_lookahead`)
- **启用**：根据速度自动调整前视距离，高速时增大，低速时减小
- **禁用**：使用固定的前视距离

## Topic 接口

### 订阅
- `/corn_row_center_line` ([nav_msgs/msg/Path](file:///home/wheeltec/agribot/agribot_ws/src/centerline_extraction/include/centerline_extraction/pid_controller.hpp#L35-L35))：参考路径
- `/odom_combined` ([nav_msgs/msg/Odometry](file:///home/wheeltec/agribot/agribot_ws/src/centerline_extraction/include/centerline_extraction/pid_controller.hpp#L36-L36))：机器人里程计

### 发布
- `/cmd_vel` ([geometry_msgs/msg/Twist](file:///home/wheeltec/agribot/agribot_ws/src/turn_on_32chassis/include/turn_on_32chassis/Quaternion_Solution.h#L9-L9))：速度控制指令
- `/lookahead_point_marker` ([visualization_msgs/msg/Marker](file:///home/wheeltec/agribot/agribot_ws/src/centerline_extraction/include/centerline_extraction/pid_controller.hpp#L38-L38))：前视点可视化（调试模式）

## 调试技巧

### 1. RViz 可视化
在 RViz 中添加以下显示项：
- **Path**：显示 `/corn_row_center_line`
- **Marker**：显示 `/lookahead_point_marker`
- **RobotModel**：显示机器人模型
- **Odometry**：显示 `/odom_combined`

### 2. 参数动态调整
使用 `ros2 param` 实时调整参数：
```bash
# 查看当前参数
ros2 param /pure_pursuit_controller list

# 调整前视距离
ros2 param /pure_pursuit_controller set lookahead_distance 0.6

# 调整线速度
ros2 param /pure_pursuit_controller set linear_velocity 0.35
```

### 3. 日志监控
```bash
# 查看详细日志
ros2 run centerline_extraction pure_pursuit_controller --ros-args --log-level debug
```

## 与 PID 控制器对比

| 特性 | Pure Pursuit | PID |
|------|--------------|-----|
| **实现难度** | 简单 | 中等 |
| **参数数量** | 少（主要调前视距离） | 多（6 个 PID 参数） |
| **抗噪性** | 好（天然滤波） | 依赖微分项处理 |
| **曲线跟踪** | 优秀 | 良好 |
| **计算量** | 小 | 小 |
| **适用场景** | 农业、非结构化环境 | 结构化环境 |

## 故障排除

### 问题 1：小车左右震荡
**原因**：前视距离太小或速度太快  
**解决**：增大 `lookahead_distance` 或降低 `linear_velocity`

### 问题 2：响应迟缓，跟不上路径变化
**原因**：前视距离太大  
**解决**：减小 `lookahead_distance` 或启用 `adaptive_lookahead`

### 问题 3：无法跟踪急转弯
**原因**：角速度限制太严格  
**解决**：增大 `max_angular_velocity`（注意机械限制）

### 问题 4：横向误差大
**原因**：速度太快或前视距离不合适  
**解决**：降低速度，调整 `lookahead_distance`，启用 `adaptive_lookahead`

## 完整工作流程

1. **启动传感器和底盘驱动**
   ```bash
   ros2 launch turn_on_agribot turn_on_agribot.launch.py
   ```

2. **启动中心线检测**
   ```bash
   ros2 run centerline_extraction corn_row_detector_projection
   ```

3. **启动 Pure Pursuit 控制器**
   ```bash
   ros2 launch centerline_extraction pure_pursuit.launch.py
   ```

4. **在 RViz 中监控**
   ```bash
   rviz2
   ```

## 进阶配置

### 自定义速度曲线
根据作物行情况自定义速度曲线：
```python
# 在 launch 文件中配置
parameters=[
    {'linear_velocity': 0.3},
    {'velocity_gain': 0.8},  # 增大增益，误差大时更明显减速
]
```

### 禁用自适应功能
在平整地面上使用固定参数：
```python
parameters=[
    {'adaptive_lookahead': False},
    {'lookahead_distance': 0.6},
]
```

## 参考文献

1. Coulter, R. C. (1992). *Implementation of the Pure Pursuit Path Tracking Algorithm*. CMU.
2. Snider, J. M. (2009). *Automatic Steering Methods for Autonomous Automobile Path Tracking*. CMU.

## 维护者

如有问题请提交 issue 或联系开发团队。
