#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController();

private:
    // 回调函数
    void center_line_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 核心控制函数
    geometry_msgs::msg::PointStamped find_lookahead_point();
    double calculate_lateral_error();
    geometry_msgs::msg::Twist calculate_control_command();

    // 辅助函数
    void get_parameters();
    void publish_lookahead_marker(const geometry_msgs::msg::PointStamped& point);

    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_line_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;

    // 控制器参数
    double lookahead_base_;         // 基础前视距离
    double lookahead_gain_;         // 前视距离速度系数
    double max_linear_speed_;       // 最大线速度
    double min_linear_speed_;       // 最小线速度
    double max_angular_speed_;      // 最大角速度
    double wheel_base_;             // 轮距（米）
    double lateral_error_gain_;     // 横向偏差校正系数
    double curve_decay_gain_;       // 弯道减速系数
    bool debug_mode_;               // 调试模式

    // 状态变量
    nav_msgs::msg::Path center_line_;  // 中心线路径
    bool has_center_line_;             // 是否收到中心线
    double current_x_;                 // 当前x坐标
    double current_y_;                 // 当前y坐标
    double current_yaw_;               // 当前偏航角
    double current_linear_vel_;        // 当前线速度
    geometry_msgs::msg::PointStamped closest_point_;  // 路径上最近的点
    geometry_msgs::msg::PointStamped lookahead_point_; // 前视目标点
};

#endif  // PURE_PURSUIT_CONTROLLER_HPP
