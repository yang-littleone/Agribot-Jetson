#include "centerline_extraction/pure_pursuit_controller.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller"), has_center_line_(false)
{
    // 订阅玉米行中心线（路径）
    center_line_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/corn_row_center_line", 10,
        std::bind(&PurePursuitController::center_line_callback, this, std::placeholders::_1));
    
    // 订阅里程计信息（小车位姿）
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1));
    
    // 发布速度控制指令
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);
    
    // 发布目标点可视化标记（调试用）
    lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/lookahead_point", 10);

    // 声明并初始化参数（适配差速小车）
    this->declare_parameter("lookahead_base", 0.4);         // 基础前视距离（米）
    this->declare_parameter("lookahead_gain", 1.5);         // 前视距离速度系数
    this->declare_parameter("max_linear_speed", 0.5);       // 最大线速度（米/秒）
    this->declare_parameter("min_linear_speed", 0.1);       // 最小线速度（米/秒）
    this->declare_parameter("max_angular_speed", 1.5);      // 最大角速度（弧度/秒）
    this->declare_parameter("wheel_base", 0.206);             // 轮距（米，需根据实际小车填写）
    this->declare_parameter("lateral_error_gain", 0.1);    // 横向偏差校正系数
    this->declare_parameter("curve_decay_gain", 0.3);       // 弯道减速系数
    this->declare_parameter("debug_mode", false);           // 调试模式开关

    // 读取参数
    get_parameters();

    // 初始化状态变量
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_yaw_ = 0.0;
    current_linear_vel_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "纯追踪控制器初始化完成（轮距: %.2f米）", wheel_base_);
}

void PurePursuitController::get_parameters()
{
    this->get_parameter("lookahead_base", lookahead_base_);
    this->get_parameter("lookahead_gain", lookahead_gain_);
    this->get_parameter("max_linear_speed", max_linear_speed_);
    this->get_parameter("min_linear_speed", min_linear_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("lateral_error_gain", lateral_error_gain_);
    this->get_parameter("curve_decay_gain", curve_decay_gain_);
    this->get_parameter("debug_mode", debug_mode_);
}

void PurePursuitController::center_line_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) {
        has_center_line_ = false;
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_WARN(this->get_logger(), "收到空的中心线，发布停止指令");
        return;
    }
    
    center_line_ = *msg;
    has_center_line_ = true;
    RCLCPP_DEBUG(this->get_logger(), "收到中心线（%zu个点）", center_line_.poses.size());
}

void PurePursuitController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 更新当前位姿
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_linear_vel_ = msg->twist.twist.linear.x;

    // 从四元数解析偏航角（yaw）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);

    // 有中心线时计算控制指令
    if (has_center_line_ && !center_line_.poses.empty()) {
        auto cmd_vel = calculate_control_command();
        cmd_vel_pub_->publish(cmd_vel);
    } else if (!has_center_line_) {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "未收到中心线，发布停止指令");
    }
}

geometry_msgs::msg::PointStamped PurePursuitController::find_lookahead_point()
{
    geometry_msgs::msg::PointStamped lookahead_point;
    lookahead_point.header.frame_id = center_line_.header.frame_id;
    lookahead_point.header.stamp = this->now();

    // 计算动态前视距离（基础距离 + 速度相关部分）
    double dynamic_lookahead = lookahead_base_ + lookahead_gain_ * current_linear_vel_;
    dynamic_lookahead = std::clamp(dynamic_lookahead, 0.5, 2.0);  // 限制范围

    // 1. 找到路径上离小车最近的点
    size_t closest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < center_line_.poses.size(); ++i) {
        const auto& p = center_line_.poses[i].pose.position;
        double dx = p.x - current_x_;
        double dy = p.y - current_y_;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    closest_point_.point = center_line_.poses[closest_idx].pose.position;

    // 2. 从最近点开始，寻找第一个超出前视距离的点（只看小车前方）
    for (size_t i = closest_idx; i < center_line_.poses.size(); ++i) {
        const auto& p = center_line_.poses[i].pose.position;
        double dx = p.x - current_x_;
        double dy = p.y - current_y_;

        // 过滤小车后方的点（仅考虑前方目标）
        double x_veh = dx * cos(current_yaw_) + dy * sin(current_yaw_);  // 小车坐标系x（前向）
        if (x_veh < 0) continue;

        // 计算到目标点的距离
        double dist = std::hypot(dx, dy);
        if (dist >= dynamic_lookahead) {
            // 若当前点刚好超过前视距离，在当前点与上一点间插值
            if (i == 0) {
                lookahead_point.point = p;
                return lookahead_point;
            }
            const auto& p_prev = center_line_.poses[i-1].pose.position;
            double dx_prev = p_prev.x - current_x_;
            double dy_prev = p_prev.y - current_y_;
            double dist_prev = std::hypot(dx_prev, dy_prev);

            // 避免除零错误
            if (std::abs(dist - dist_prev) < 1e-6) {
                lookahead_point.point = p;
                return lookahead_point;
            }

            // 线性插值找到精确前视距离点
            double t = (dynamic_lookahead - dist_prev) / (dist - dist_prev);
            lookahead_point.point.x = p_prev.x + t * (p.x - p_prev.x);
            lookahead_point.point.y = p_prev.y + t * (p.y - p_prev.y);
            lookahead_point.point.z = 0.0;
            return lookahead_point;
        }
    }

    // 若所有点都在视距内，取最后一个点
    lookahead_point.point = center_line_.poses.back().pose.position;
    return lookahead_point;
}

double PurePursuitController::calculate_lateral_error()
{
    // 计算最近点在小车坐标系下的横向偏差（左正右负）
    double dx = closest_point_.point.x - current_x_;
    double dy = closest_point_.point.y - current_y_;
    return -sin(current_yaw_) * dx + cos(current_yaw_) * dy;
}

void PurePursuitController::publish_lookahead_marker(const geometry_msgs::msg::PointStamped& point)
{
    visualization_msgs::msg::Marker marker;
    marker.header = point.header;
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point.point;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    lookahead_marker_pub_->publish(marker);
}

geometry_msgs::msg::Twist PurePursuitController::calculate_control_command()
{
    geometry_msgs::msg::Twist cmd_vel;

    // 调试模式：输出固定速度测试转向
    if (debug_mode_) {
        cmd_vel.linear.x = 0.1;
        cmd_vel.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "调试模式：线速度=0.1m/s, 角速度=0");
        return cmd_vel;
    }

    // 1. 寻找前视目标点
    lookahead_point_ = find_lookahead_point();
    publish_lookahead_marker(lookahead_point_);  // 可视化目标点

    // 2. 将目标点转换到小车坐标系
    double dx = lookahead_point_.point.x - current_x_;
    double dy = lookahead_point_.point.y - current_y_;
    double x_veh = dx * cos(current_yaw_) + dy * sin(current_yaw_);  // 前向距离
    double y_veh = -dx * sin(current_yaw_) + dy * cos(current_yaw_); // 横向距离（左正）

    // 3. 计算曲率（纯追踪核心公式）
    double curvature = 0.0;
    double denominator = x_veh * x_veh + y_veh * y_veh;
    if (std::abs(denominator) > 1e-6) {
        curvature = (2 * y_veh) / denominator;  // 曲率 = 2*横向距离 / 距离平方
    }

    // 4. 加入横向偏差校正（减少稳态误差）
    double lateral_error = calculate_lateral_error();
    curvature += lateral_error_gain_ * lateral_error;

    // 5. 计算角速度（曲率 * 线速度）
    double angular_vel = current_linear_vel_ * curvature;
    angular_vel = std::clamp(angular_vel, -max_angular_speed_, max_angular_speed_);

    // 6. 计算线速度（弯道减速）
    double linear_speed;
    if (std::abs(angular_vel) < 0.2) {  // 小转弯不减速
        linear_speed = max_linear_speed_;
    } else {
        // 角速度越大，速度越低（保留基础速度）
        linear_speed = max_linear_speed_ * (1.0 - curve_decay_gain_ * std::abs(angular_vel)/max_angular_speed_);
    }
    linear_speed = std::max(linear_speed, min_linear_speed_);  // 不低于最小速度

    // 7. 赋值控制指令
    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = angular_vel;

    // 调试日志
    RCLCPP_DEBUG(this->get_logger(), 
        "前视距离: %.2f, 横向偏差: %.2f, 线速度: %.2f, 角速度: %.2f",
        std::hypot(dx, dy), lateral_error, linear_speed, angular_vel);

    return cmd_vel;
}
