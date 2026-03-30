#!/bin/bash

# 测试直线生成器快速启动脚本
# 用于测试 Pure Pursuit 路径跟踪算法

echo "======================================"
echo "  测试直线生成器 - 快速启动脚本"
echo "======================================"
echo ""

# 设置默认参数
DEFAULT_LINE_SLOPE=0.1
DEFAULT_LINE_INTERCEPT=0.0
DEFAULT_START_X=0.5
DEFAULT_END_X=3.0

echo "默认参数配置:"
echo "  直线斜率 (line_slope): $DEFAULT_LINE_SLOPE"
echo "  截距 (line_intercept): $DEFAULT_LINE_INTERCEPT"
echo "  起始 X (start_x): $DEFAULT_START_X m"
echo "  结束 X (end_x): $DEFAULT_END_X m"
echo ""

# 检查是否已 source setup.bash
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "错误：未检测到 ROS 2 环境!"
    echo "请先运行：source ~/agribot/agribot_ws/install/setup.bash"
    exit 1
fi

echo "ROS 2 环境检测通过 ✓"
echo ""

# 显示帮助信息
show_help() {
    echo "使用方法:"
    echo "  ./run_test_line.sh [选项]"
    echo ""
    echo "选项:"
    echo "  -s, --slope VALUE      设置直线斜率 (默认：$DEFAULT_LINE_SLOPE)"
    echo "  -i, --intercept VALUE  设置截距 (默认：$DEFAULT_LINE_INTERCEPT)"
    echo "  -x, --start-x VALUE    设置起始 X 坐标 (默认：$DEFAULT_START_X)"
    echo "  -e, --end-x VALUE      设置结束 X 坐标 (默认：$DEFAULT_END_X)"
    echo "  -h, --help             显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  # 使用默认参数运行"
    echo "  ./run_test_line.sh"
    echo ""
    echo "  # 设置斜率为 0.3，截距为 -0.2"
    echo "  ./run_test_line.sh -s 0.3 -i -0.2"
    echo ""
    echo "  # 设置路径范围为 0.3m 到 4.0m"
    echo "  ./run_test_line.sh -x 0.3 -e 4.0"
    echo ""
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -s|--slope)
            LINE_SLOPE="$2"
            shift 2
            ;;
        -i|--intercept)
            LINE_INTERCEPT="$2"
            shift 2
            ;;
        -x|--start-x)
            START_X="$2"
            shift 2
            ;;
        -e|--end-x)
            END_X="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "未知选项：$1"
            show_help
            exit 1
            ;;
    esac
done

# 使用默认值（如果未提供）
LINE_SLOPE=${LINE_SLOPE:-$DEFAULT_LINE_SLOPE}
LINE_INTERCEPT=${LINE_INTERCEPT:-$DEFAULT_LINE_INTERCEPT}
START_X=${START_X:-$DEFAULT_START_X}
END_X=${END_X:-$DEFAULT_END_X}

echo "启动测试直线生成器..."
echo ""

# 启动节点并在后台运行
ros2 run centerline_extraction test_line_generator &
NODE_PID=$!

# 等待节点启动
sleep 2

# 检查节点是否成功启动
if ps -p $NODE_PID > /dev/null; then
    echo "✓ 节点已成功启动 (PID: $NODE_PID)"
    echo ""
    
    # 设置参数
    echo "正在配置参数..."
    ros2 param set /test_line_generator line_slope $LINE_SLOPE
    ros2 param set /test_line_generator line_intercept $LINE_INTERCEPT
    ros2 param set /test_line_generator start_x $START_X
    ros2 param set /test_line_generator end_x $END_X
    
    echo ""
    echo "======================================"
    echo "  测试直线生成器运行中"
    echo "======================================"
    echo ""
    echo "当前配置:"
    echo "  直线斜率：$LINE_SLOPE"
    echo "  截距：$LINE_INTERCEPT"
    echo "  路径范围：[$START_X, $END_X] m"
    echo ""
    echo "发布的话题:"
    echo "  /corn_row_center_line (odom_combined 坐标系)"
    echo "  /corn_row_center_line_viz (base_link 坐标系)"
    echo ""
    echo "控制指令:"
    echo "  按 Ctrl+C 停止节点"
    echo ""
    echo "动态调整参数示例:"
    echo "  ros2 param set /test_line_generator line_slope 0.2"
    echo "  ros2 param set /test_line_generator line_intercept -0.1"
    echo ""
    echo "======================================"
    echo ""
    
    # 等待用户中断
    wait $NODE_PID
else
    echo "✗ 节点启动失败!"
    exit 1
fi
