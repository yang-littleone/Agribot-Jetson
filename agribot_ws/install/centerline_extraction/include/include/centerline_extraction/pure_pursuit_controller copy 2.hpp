#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"  // 使用普通Twist，不使用Stamped
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController();
    
private:
    void center_line_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void getCurrentPose(const nav_msgs::msg::Odometry::SharedPtr odom,
                       double& x, double& y, double& yaw);
    geometry_msgs::msg::Twist calculateControlCommand();  // 返回普通Twist
    geometry_msgs::msg::PointStamped findLookaheadPoint();
    double calculateLateralError();
    void get_parameters();
    
    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_line_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  // 发布普通Twist
    
    // 数据存储
    nav_msgs::msg::Path center_line_;
    bool has_center_line_;
    double current_x_, current_y_, current_yaw_;
    double current_linear_vel_;
    geometry_msgs::msg::PointStamped closest_point_;
    geometry_msgs::msg::PointStamped lookahead_point_;
    
    // 控制器参数（与配置文件匹配）
    double lookahead_distance_;
    double lookahead_gain_;       // 添加预瞄增益参数
    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;
    double wheel_base_;  // 必须与配置中的wheel_separation一致
    double kp_lateral_;
    double kp_curve_;     // 添加曲线减速参数
    bool debug_mode_;
};

#endif  // PURE_PURSUIT_CONTROLLER_HPP