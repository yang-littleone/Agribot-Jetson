#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PIDController : public rclcpp::Node
{
public:
    PIDController();

private:
    // 回调函数
    void center_line_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 核心控制函数
    geometry_msgs::msg::PointStamped find_target_point();
    geometry_msgs::msg::Twist calculate_control_command();

    // PID辅助函数
    double compute_pid(double error, double dt, double &integral, double &previous_error, 
                      double kp, double ki, double kd);

    // 辅助函数
    void get_parameters();
    void publish_target_marker(const geometry_msgs::msg::PointStamped& point);

    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_line_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;

    // 控制器参数
    double target_distance_;        // 目标跟随距离（距离路径前方多少米）
    double max_linear_speed_;       // 最大线速度
    double min_linear_speed_;       // 最小线速度
    double max_angular_speed_;      // 最大角速度
    
    // PID参数
    double lateral_kp_;             // 横向控制比例系数
    double lateral_ki_;             // 横向控制积分系数
    double lateral_kd_;             // 横向控制微分系数
    
    double heading_kp_;             // 航向控制比例系数
    double heading_ki_;             // 航向控制积分系数
    double heading_kd_;             // 航向控制微分系数

    bool debug_mode_;               // 调试模式

    // 状态变量
    nav_msgs::msg::Path center_line_;  // 中心线路径
    bool has_center_line_;             // 是否收到中心线
    double current_x_;                 // 当前x坐标
    double current_y_;                 // 当前y坐标
    double current_yaw_;               // 当前偏航角
    double current_linear_vel_;        // 当前线速度
    double current_angular_vel_;       // 当前角速度
    
    geometry_msgs::msg::PointStamped target_point_; // 目标点
    
    // PID控制器状态变量
    double lateral_integral_;
    double lateral_previous_error_;
    double heading_integral_;
    double heading_previous_error_;
    
    // 时间记录
    rclcpp::Time last_time_;
    bool first_run_;

};

#endif  // PID_CONTROLLER_HPP