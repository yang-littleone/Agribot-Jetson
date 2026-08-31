# 大论文田间试验工具

本目录只包含试验编排、现场登记和离线统计工具。感知、控制、定位算法及基础 YAML 保持小论文冻结版本；仅增强 `record_field_bag.sh` 和 `validate_field_trial.py` 的大论文录包/验收范围。正式试验前运行 `scripts/verify_frozen_code.sh` 检查该边界。

现场按小论文相同的三窗口方式操作，命令见 [现场简明操作.md](现场简明操作.md)。论文试验设计、真值测量和统计口径再查看 [大论文正式试验执行手册.md](大论文正式试验执行手册.md)。

没有作物行的室内环境使用 [室内掉头测试.md](室内掉头测试.md)，不要启动真实玉米行检测器。

主要文件：

- `thesis_field_experiments/launch/headland_path_tracking.launch.py`：窗口三使用的大论文掉头启动文件。
- `thesis_field_experiments/launch/indoor_headland_test.launch.py`：两条 1 m 模拟中心线的室内掉头测试。
- `config/trial_matrix.yaml`：正式试验数量和固定参数。
- `scripts/validate_trial.py`：一次试验结束后的数据完整性检查。
- `scripts/analyze_trials.py`：自动日志与人工登记表合并统计。
- `templates/trial_register.csv`：现场主登记表表头；复制后填写，勿直接覆盖模板。

所有命令默认从工作空间 `/home/wheeltec/agribot/agribot_ws` 执行。
