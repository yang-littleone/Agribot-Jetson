# 玉米冠下感知质量约束导航：田间试验规程

本文只验证“内侧双行稳健中心线—可解释质量评价—质量约束控制”。地头掉头和
障碍避让均不进入本文贡献；田间配置已关闭控制器自动掉头，并明确禁用未接入控制
闭环的障碍检测节点。

## 1. 正式试验前的参数锁定

先用不进入正式统计的正常行摸底数据完成以下工作：

1. 实测车体最大外廓宽度、每个试验行段的行距和要求的植株安全间隙；
2. 确定 ROI、高度范围、最小点数、行距容差、拟合残差阈值、跳变阈值、丢线
   保持时间、置信度分级和停车阈值；
3. 架空驱动轮后在 0.05 m/s 指令下确认转向符号和停车逻辑；落地摸底使用能够克服
   底盘死区的 0.18 m/s 名义速度；
4. 将 `config/field_row_follow.yaml` 复制为带日期的锁定配置，计算 SHA256，并保存
   到试验台账。正式试验开始后不得修改；缺株、遮挡、杂草行只用于独立测试。

危险横向误差阈值必须逐行段计算：

```text
E_safe = 实测行距/2 − 实测车宽/2 − 植株安全间隙
```

检测器同时发布 `/corridor_error_budget`，其值用该帧实际行距计算。若
`E_safe <= 0`，该行段不具备几何通行条件，不得启动自动行走。

## 2. 启动与安全检查

终端1启动底盘、传感器、融合里程计和中心线感知：

```bash
ros2 launch centerline_extraction field_system_with_perception.launch.py
```

确认 `/livox/lidar`、`/odometry/filtered`、TF、硬件急停和遥控接管均正常。人员不得
站在车体前方或两行植株之间，同时确认 `/corn_row_center_line` 和五个质量分量持续
更新。首次只允许架空轮测试，确认方向正确后再进行落地短距离摸底。

终端2单独启动本次路径跟踪和记录。每次试验结束后只需停止并重新运行这个启动，
基础系统和中心线感知不需要重启：

```bash
ros2 launch centerline_extraction field_path_tracking.launch.py \
  trial_id:=complex_qaware_v018_r1 scenario:=complex \
  quality_aware:=true max_linear_speed:=0.18 max_distance:=0.0 \
  repeat_index:=1
```

`quality_aware:=false` 是固定名义速度 PID 对照；两组使用相同PID增益和终端1中
持续运行的同一感知方法，只改变质量是否作用于速度/角速度、历史中心线保持和停车。
当前5 m场地不使用里程计距离自动停车，`max_distance:=0.0`表示禁用距离上限；
到达场地终点后人工停车。180 s时间上限、中心线/质量超时停车和安全裕度停车仍然
有效。

记录器为每次运行生成 `timeseries.csv` 和 `metadata.json`。后者包含感知、控制参数
快照；CSV 保留点数、观测完整性、行距、残差、安全裕度、误差预算、五个质量分量、
最终置信度、控制质量因子、独立横向/航向控制误差、里程计与中心线数据年龄及停车
状态，不能只保留最终置信度。

每次正式试验另开一个终端运行录包脚本，`trial_id` 必须与路径跟踪启动命令
完全一致：

```bash
ros2 run centerline_extraction record_field_bag.sh complex_qaware_v018_r1
```

脚本将原始点云、IMU、轮速/LIO/融合里程计、中心线、质量分量、控制输出和地头
状态优先保存到已挂载U盘的
`agribot_field_data/field_trial_bags/日期时间_trial_id/`。U盘未插入、不可写或
剩余空间小于1 GiB时自动使用工作空间的 `field_trial_bags/`；U盘在录制中写满或
异常断开时，脚本在工作空间建立 `continued_after_usb` 目录续录。按 `Ctrl+C`
正常结束后，对终端打印出的实际目录执行 `ros2 bag info "实际目录"` 检查消息数。

成熟期玉米冠层下不要求外部视频覆盖全程。20～50 m精度试验段采用高于冠层的
RTK天线、自动跟踪全站仪，或每隔2～5 m测量车辆中心轨迹相对双行中心的离散真值；
200～500 m长距离试验只评价完成率、植株接触、人工干预、丢线与停车持续时间。
真值统一写入`docs/ground_truth_template.csv`，时间基准须与记录器对齐。车辆两侧
宜安装柔性接触开关，区分叶片轻触和茎秆碰撞。

## 3. 感知消融

对同一份 rosbag 依次运行以下四组，回放速率和起止时间完全相同。回放时不启动
底盘控制节点：

| `perception_method` | 内侧行 | 稳健细化 | 平行双行 | 时序约束 | 质量输出 |
|---|---:|---:|---:|---:|---:|
| `ls_baseline` | 0 | 0 | 0 | 0 | 0 |
| `inner_robust` | 1 | 1 | 0 | 0 | 0 |
| `parallel_temporal` | 1 | 1 | 1 | 1 | 0 |
| `full` | 1 | 1 | 1 | 1 | 1 |

示例（基线）：

```bash
ros2 launch centerline_extraction perception_ablation.launch.py \
  trial_id:=bag01 scenario:=complex perception_method:=ls_baseline \
  enable_innermost:=false enable_robust:=false use_parallel:=false \
  enable_temporal:=false enable_quality:=false
```

其余三组按表切换参数，并用 `ros2 bag play BAG_NAME --clock` 回放。同一 rosbag
的四种方法必须使用同一个 `trial_id`，以便共享同一份真值；记录目录带独立时间戳，
不会互相覆盖。报告中心线横向 MAE、RMSE、P95、最大误差、航向误差、有效检测率
和帧间抖动。

## 4. 闭环对照矩阵

正常行和独立复杂行（缺株/遮挡/杂草）各测试：

- 固定名义速度 PID、质量约束 PID；
- 核心矩阵最大线速度统一为 0.18 m/s；
- 每一组合至少 5 次。

摸底试验表明 0.10 m/s 不足以克服实车低速死区，车辆不能形成稳定连续运动，因此
不把 0.10 m/s 数据作为正式对照，也不据此声称算法具有多速度鲁棒性。最低正式
闭环试验量为 `2 行况 × 2 控制 × 1 速度 × 5 次 = 20 次`。交替或随机化运行顺序，
避免电池电量、土壤变化和光照时间与某一控制方法绑定。试验段、起始位置、轮胎
气压和载荷保持一致。

如需要验证速度适应性，在核心20次之外增加0.40 m/s最大线速度的同结构矩阵，
完整增强试验量为40次。质量约束会降低实际速度，因此论文必须把0.18/0.40 m/s
称为最大速度设置，并报告实际 `/cmd_vel` 和里程计速度。完整现场步骤以
`docs/real_corn_field_paper_experiment_guide.md` 为准。

## 5. 离线统计

```bash
ros2 run centerline_extraction analyze_field_trials.py \
  --trials field_trial_results \
  --ground-truth ground_truth.csv \
  --safe-threshold 0.14 \
  --output field_analysis_report.json
```

当前锁定的行距0.60 m、车宽0.22 m和植株间隙0.05 m对应阈值0.14 m；若实测值
变化，`--safe-threshold` 必须重新按公式计算。报告包含感知消融误差、
置信度与误差 Spearman 相关、风险 ROC-AUC/平均精确率及曲线点、高中低置信区间、
五个权重分别 ±20% 的单因素敏感性，以及闭环完成率、P95/最大误差、接触、干预、
丢线时长和运行时间。置信度若不能稳定区分安全与危险中心线，不得把质量评价写成
已验证创新。
