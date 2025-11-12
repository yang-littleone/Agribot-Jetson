#ifndef PURE_PURSUIT_CONTROLLER_HPP
#define PURE_PURSUIT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController();
    
private:
    void centerLineCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    
    // 计算当前位置与目标路径的预瞄点
    geometry_msgs::msg::PoseStamped findLookaheadPoint();
    
    // 从里程计数据获取当前姿态（x, y, yaw）
    void getCurrentPose(const nav_msgs::msg::Odometry::SharedPtr msg, 
                       float& x, float& y, float& yaw);
    
    // 计算两点之间的距离
    float distanceBetweenPoints(float x1, float y1, float x2, float y2);
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_line_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    nav_msgs::msg::Path current_path_;
    nav_msgs::msg::Odometry current_odom_;
    bool path_received_;
    bool odom_received_;
    
    // 控制参数
    float lookahead_distance_;  // 预瞄距离
    float wheelbase_;           // 轴距
    float max_linear_speed_;    // 最大线速度
    float max_angular_speed_;   // 最大角速度
    float control_frequency_;   // 控制频率
};

#endif // PURE_PURSUIT_CONTROLLER_HPP
