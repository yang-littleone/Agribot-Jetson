#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "centerline_extraction/pure_pursuit_controller.hpp"

PurePursuitController::PurePursuitController() : Node("pure_pursuit_controller")
{
    // 创建订阅者和发布者
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/corn_row_center_line", 10, 
        std::bind(&PurePursuitController::path_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_combined", 10, 
        std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1));
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    lookahead_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_point_marker", 10);

    // 声明参数
    this->declare_parameter<double>("lookahead_distance", 0.5);
    this->declare_parameter<double>("min_lookahead_distance", 0.3);
    this->declare_parameter<double>("max_lookahead_distance", 1.0);
    this->declare_parameter<double>("linear_velocity", 0.3);
    this->declare_parameter<double>("max_linear_velocity", 0.5);
    this->declare_parameter<double>("min_linear_velocity", 0.1);
    this->declare_parameter<double>("max_angular_velocity", 1.0);
    this->declare_parameter<double>("velocity_gain", 0.5);
    this->declare_parameter<bool>("adaptive_lookahead", false);
    this->declare_parameter<bool>("debug_mode", true);

    // 获取参数
    this->get_parameter("lookahead_distance", lookahead_distance_);
    this->get_parameter("min_lookahead_distance", min_lookahead_distance_);
    this->get_parameter("max_lookahead_distance", max_lookahead_distance_);
    this->get_parameter("linear_velocity", linear_velocity_);
    this->get_parameter("max_linear_velocity", max_linear_velocity_);
    this->get_parameter("min_linear_velocity", min_linear_velocity_);
    this->get_parameter("max_angular_velocity", max_angular_velocity_);
    this->get_parameter("velocity_gain", velocity_gain_);
    this->get_parameter("adaptive_lookahead", adaptive_lookahead_);
    this->get_parameter("debug_mode", debug_mode_);

    // 初始化状态变量
    has_path_ = false;
    robot_x_ = 0.0;
    robot_y_ = 0.0;
    robot_yaw_ = 0.0;
    robot_linear_vel_ = 0.0;
    first_run_ = true;
    last_time_ = this->now();

    // 创建控制循环定时器（50Hz）
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&PurePursuitController::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Parameters: lookahead=%.2fm, velocity=%.2fm/s, max_omega=%.2frad/s", 
                lookahead_distance_, linear_velocity_, max_angular_velocity_);
}

PurePursuitController::~PurePursuitController()
{
}

void PurePursuitController::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Received empty path");
        has_path_ = false;
        return;
    }

    current_path_ = *msg;
    has_path_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Received path with %zu poses", msg->poses.size());
}

void PurePursuitController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    
    // 从四元数计算偏航角
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot_yaw_ = yaw;
    
    robot_linear_vel_ = msg->twist.twist.linear.x;
}

void PurePursuitController::control_loop()
{
    // 检查是否有有效路径
    if (!has_path_ || current_path_.poses.empty())
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No valid path available");
        
        // 发布零速度命令
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    // 计算时间差
    rclcpp::Time current_time = this->now();
    
    if (first_run_)
    {
        last_time_ = current_time;
        first_run_ = false;
        return;
    }

    // 查找前视点
    int lookahead_idx = find_lookahead_point();
    
    if (lookahead_idx < 0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not find lookahead point");
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    // 计算路径曲率
    double curvature = calculate_curvature(lookahead_idx);
    
    // 计算控制指令
    geometry_msgs::msg::Twist cmd = calculate_control_command();
    
    // 限制角速度
    if (std::abs(cmd.angular.z) > max_angular_velocity_)
    {
        cmd.angular.z = (cmd.angular.z > 0) ? max_angular_velocity_ : -max_angular_velocity_;
    }
    
    // 发布速度命令
    cmd_vel_pub_->publish(cmd);
    
    // 发布可视化标记
    if (debug_mode_)
    {
        publish_lookahead_marker(lookahead_point_);
    }
    
    // 记录日志
    RCLCPP_DEBUG(this->get_logger(), 
                 "Control: v=%.2f m/s, omega=%.2f rad/s, curvature=%.2f 1/m, lookahead_idx=%d",
                 cmd.linear.x, cmd.angular.z, curvature, lookahead_idx);
    
    last_time_ = current_time;
}

int PurePursuitController::find_lookahead_point()
{
    if (current_path_.poses.empty())
    {
        return -1;
    }

    double best_dist = -1.0;
    int lookahead_idx = -1;
    
    // 遍历路径点，找到距离机器人当前位置为 lookahead_distance 的点
    for (size_t i = 0; i < current_path_.poses.size(); i++)
    {
        double dx = current_path_.poses[i].pose.position.x - robot_x_;
        double dy = current_path_.poses[i].pose.position.y - robot_y_;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        // 找到第一个距离大于 lookahead_distance 的点
        if (dist >= lookahead_distance_)
        {
            // 如果是第一个满足条件的点，或者比之前的点更接近 lookahead_distance
            if (lookahead_idx < 0 || dist < best_dist)
            {
                best_dist = dist;
                lookahead_idx = i;
            }
        }
    }
    
    // 如果没有找到满足条件的点，使用最后一个点
    if (lookahead_idx < 0)
    {
        lookahead_idx = current_path_.poses.size() - 1;
        RCLCPP_DEBUG(this->get_logger(), "Using last path point as lookahead point");
    }
    
    // 保存前视点信息
    lookahead_point_.x = current_path_.poses[lookahead_idx].pose.position.x;
    lookahead_point_.y = current_path_.poses[lookahead_idx].pose.position.y;
    lookahead_point_.z = current_path_.poses[lookahead_idx].pose.position.z;
    
    return lookahead_idx;
}

double PurePursuitController::calculate_curvature(int lookahead_idx)
{
    if (lookahead_idx < 0 || lookahead_idx >= static_cast<int>(current_path_.poses.size()))
    {
        return 0.0;
    }

    // 将前视点转换到机器人坐标系
    double dx = lookahead_point_.x - robot_x_;
    double dy = lookahead_point_.y - robot_y_;
    
    // 旋转到机器人坐标系
    double x_robot = dx * std::cos(-robot_yaw_) - dy * std::sin(-robot_yaw_);
    double y_robot = dx * std::sin(-robot_yaw_) + dy * std::cos(-robot_yaw_);
    
    // 计算曲率：kappa = 2 * y / (L^2)，其中 L 是前视距离
    // 这是 Pure Pursuit 的核心公式
    double L = std::sqrt(x_robot * x_robot + y_robot * y_robot);
    
    if (L < 0.1) // 避免除零和数值不稳定
    {
        return 0.0;
    }
    
    double curvature = 2.0 * y_robot / (L * L);
    
    return curvature;
}

geometry_msgs::msg::Twist PurePursuitController::calculate_control_command()
{
    geometry_msgs::msg::Twist cmd;
    
    // 计算曲率
    int lookahead_idx = find_lookahead_point();
    double curvature = calculate_curvature(lookahead_idx);
    
    // Pure Pursuit 控制律：omega = v * kappa
    double angular_velocity = linear_velocity_ * curvature;
    
    // 自适应线速度：根据横向误差调整速度
    double lateral_error = std::abs(lookahead_point_.y - robot_y_);
    double adjusted_velocity = linear_velocity_;
    
    if (adaptive_lookahead_)
    {
        // 横向误差大时减速
        adjusted_velocity = linear_velocity_ * (1.0 - velocity_gain_ * std::min(1.0, lateral_error / 2.0));
        adjusted_velocity = std::max(min_linear_velocity_, std::min(max_linear_velocity_, adjusted_velocity));
        
        // 自适应调整前视距离：速度越快，前视距离越远
        lookahead_distance_ = min_lookahead_distance_ + 
                             (max_lookahead_distance_ - min_lookahead_distance_) * 
                             (adjusted_velocity - min_linear_velocity_) / 
                             (max_linear_velocity_ - min_linear_velocity_);
    }
    
    cmd.linear.x = adjusted_velocity;
    cmd.angular.z = angular_velocity;
    
    return cmd;
}

double PurePursuitController::calculate_yaw_from_path()
{
    if (current_path_.poses.size() < 2)
    {
        return robot_yaw_;
    }
    
    // 找到最接近机器人的路径点
    double min_dist = 1e6;
    int closest_idx = 0;
    
    for (size_t i = 0; i < current_path_.poses.size(); i++)
    {
        double dx = current_path_.poses[i].pose.position.x - robot_x_;
        double dy = current_path_.poses[i].pose.position.y - robot_y_;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    // 使用前几个点计算期望航向
    int next_idx = std::min(closest_idx + 2, static_cast<int>(current_path_.poses.size() - 1));
    
    double dx = current_path_.poses[next_idx].pose.position.x - current_path_.poses[closest_idx].pose.position.x;
    double dy = current_path_.poses[next_idx].pose.position.y - current_path_.poses[closest_idx].pose.position.y;
    
    return std::atan2(dy, dx);
}

void PurePursuitController::publish_lookahead_marker(const geometry_msgs::msg::Point& point)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom_combined";
    marker.header.stamp = this->now();
    marker.ns = "lookahead_point";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.w = 1.0;
    
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

void PurePursuitController::get_parameters()
{
    // 参数已在构造函数中获取
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}
