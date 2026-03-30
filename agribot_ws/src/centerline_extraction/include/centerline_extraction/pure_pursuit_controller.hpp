#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController();
    ~PurePursuitController();

private:
    // 回调函数
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_loop();

    // Pure Pursuit 核心算法
    int find_lookahead_point();
    double calculate_curvature(int lookahead_idx);
    geometry_msgs::msg::Twist calculate_control_command();

    // 辅助函数
    void get_parameters();
    void publish_lookahead_marker(const geometry_msgs::msg::Point& point);
    double calculate_yaw_from_path();

    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Pure Pursuit 参数
    double lookahead_distance_;     // 前视距离（米）
    double min_lookahead_distance_; // 最小前视距离
    double max_lookahead_distance_; // 最大前视距离
    double linear_velocity_;        // 期望线速度（m/s）
    double max_linear_velocity_;    // 最大线速度
    double min_linear_velocity_;    // 最小线速度
    double max_angular_velocity_;   // 最大角速度（rad/s）
    double velocity_gain_;          // 速度增益（根据横向误差调整速度）
    bool adaptive_lookahead_;       // 是否使用自适应前视距离

    // 状态变量
    nav_msgs::msg::Path current_path_;  // 当前路径
    bool has_path_;                     // 是否收到有效路径
    double robot_x_;                    // 机器人当前位置 x
    double robot_y_;                    // 机器人当前位置 y
    double robot_yaw_;                  // 机器人当前航向角
    double robot_linear_vel_;           // 机器人当前线速度
    
    geometry_msgs::msg::Point lookahead_point_; // 前视点
    rclcpp::Time last_time_;                           // 上次控制周期时间
    bool first_run_;                                   // 是否首次运行

    // 调试参数
    bool debug_mode_;               // 调试模式，发布可视化标记
};

#endif  // PURE_PURSUIT_CONTROLLER_HPP
