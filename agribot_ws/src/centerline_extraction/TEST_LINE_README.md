# 测试直线生成器使用说明

## 概述
`test_line_generator` 是一个用于测试 Pure Pursuit 路径跟踪算法的工具。它会发布一条固定的直线，模拟玉米行中心线检测结果。

## 编译
```bash
cd ~/agribot/agribot_ws
colcon build --packages-select centerline_extraction
source install/setup.bash
```

## 运行方法

### 1. 基本运行
在一个终端中运行测试直线生成器：
```bash
ros2 run centerline_extraction test_line_generator
```

### 2. 同时运行路径跟踪控制器
在另一个终端中运行 Pure Pursuit 控制器：
```bash
ros2 run centerline_extraction pure_pursuit_controller
```

### 3. 查看可视化
在 RViz2 中添加以下话题进行可视化：
- `/corn_row_center_line` - 中心线路径（odom_combined 坐标系）
- `/corn_row_center_line_viz` - 中心线路径（base_link 坐标系）

## 可配置参数

### 默认参数值
- `line_slope`: 0.1（直线斜率）
- `line_intercept`: 0.0（直线截距）
- `start_x`: 0.5（起始 x 坐标，小车前方距离）
- `end_x`: 3.0（结束 x 坐标）
- `point_spacing`: 0.1（路径点间距）
- `publish_rate`: 10（发布频率 Hz）

### 动态调整参数
可以在运行时通过 ROS 2 参数服务器调整参数：

```bash
# 调整直线斜率（更陡的斜率）
ros2 param set /test_line_generator line_slope 0.3

# 调整截距（向左偏移）
ros2 param set /test_line_generator line_intercept -0.2

# 调整路径范围（更近的开始位置）
ros2 param set /test_line_generator start_x 0.3

# 调整路径范围（更远的结束位置）
ros2 param set /test_line_generator end_x 4.0

# 调整点间距（更密集的路径点）
ros2 param set /test_line_generator point_spacing 0.05

# 调整发布频率
ros2 param set /test_line_generator publish_rate 20
```

### 查看当前参数
```bash
ros2 param get /test_line_generator line_slope
ros2 param get /test_line_generator line_intercept
ros2 param dump /test_line_generator
```

## 测试场景

### 场景 1: 直线路径跟踪
使用默认参数，测试小车沿直线行驶的能力：
```bash
ros2 param set /test_line_generator line_slope 0.0
ros2 param set /test_line_generator line_intercept 0.0
```

### 场景 2: 轻微弯曲路径
模拟轻微的曲线行：
```bash
ros2 param set /test_line_generator line_slope 0.15
```

### 场景 3: 较大偏移
模拟需要较大转向调整的场景：
```bash
ros2 param set /test_line_generator line_slope 0.3
ros2 param set /test_line_generator line_intercept -0.3
```

### 场景 4: 近距离路径
测试近场路径跟踪性能：
```bash
ros2 param set /test_line_generator start_x 0.2
ros2 param set /test_line_generator end_x 2.0
```

### 场景 5: 远距离路径
测试远场路径跟踪性能：
```bash
ros2 param set /test_line_generator start_x 0.5
ros2 param set /test_line_generator end_x 5.0
```

## 监控与调试

### 1. 查看发布的消息
```bash
ros2 topic echo /corn_row_center_line
```

### 2. 查看节点日志
```bash
ros2 node info /test_line_generator
```

### 3. 监控控制指令
```bash
ros2 topic echo /cmd_vel
```

### 4. 查看 TF 变换
```bash
ros2 run tf2_tools view_frames.py
evince frames.pdf
```

## 预期行为

1. **直线路径** (`slope=0, intercept=0`): 
   - 小车应保持直行或微调方向

2. **倾斜路径** (`slope>0`): 
   - 小车应向右转弯跟踪路径
   - 斜率越大，转弯越急

3. **负截距** (`intercept<0`): 
   - 路径起点在小车左侧
   - 小车应向左转然后跟踪路径

4. **正截距** (`intercept>0`): 
   - 路径起点在小车右侧
   - 小车应向右转然后跟踪路径

## 故障排查

### 问题：小车不移动
- 检查是否已启动底盘驱动节点
- 确认 `/cmd_vel` 话题有消息发布
- 检查安全开关是否启用

### 问题：小车振荡严重
- 减小 `lookahead_distance` 参数（在 pure_pursuit_controller 中）
- 减小路径斜率或截距
- 检查 `max_angular_velocity` 是否过小

### 问题：路径跟踪不准确
- 增加路径点密度（减小 `point_spacing`）
- 调整 Pure Pursuit 的 `lookahead_distance`
- 检查 TF 坐标系变换是否正确

## 注意事项

1. 首次测试时，建议在空旷场地进行
2. 从小斜率、小截距开始，逐步增加难度
3. 确保小车前方有足够的空间（至少 `end_x` 指定的距离）
4. 监控电池电量，低电量可能影响控制性能
5. 如遇紧急情况，立即按下急停按钮

## 与其他节点的集成

该测试生成器可以与以下节点配合使用：
- `corn_row_detector_projection`: 真实的玉米行检测节点（可同时运行，通过话题切换）
- `pure_pursuit_controller`: Pure Pursuit 路径跟踪控制器
- `turn_on_32chassis`: 底盘驱动节点

## 扩展功能

如需更复杂的测试路径（如曲线路径），可以修改 `test_line.cpp` 中的路径生成逻辑：
- 添加二次曲线支持
- 添加分段路径
- 添加随机扰动
- 添加动态变化的路径
