#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PIDController : public rclcpp::Node
{
public:
    PIDController();

private:
    enum class NavigationMode
    {
        ROW_FOLLOW,
        U_TURN,
        NEXT_ROW_REACQUIRE
    };

    // 回调函数
    void center_line_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void confidence_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void safety_margin_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void headland_detected_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // 核心控制函数
    geometry_msgs::msg::PointStamped find_target_point();
    geometry_msgs::msg::Twist calculate_control_command();

    // PID辅助函数
    double compute_pid(double error, double dt, double &integral, double &previous_error, 
                      double kp, double ki, double kd);

    // 辅助函数
    void get_parameters();
    void publish_target_marker(const geometry_msgs::msg::PointStamped& point);
    double compute_confidence_factor() const;
    double compute_safety_factor() const;
    bool should_stop_for_safety() const;
    double normalize_angle(double angle) const;
    void reset_pid_state();
    void update_travel_distance(double x, double y);
    bool should_start_headland_turn() const;
    void start_headland_turn();
    void start_next_row_reacquire();
    bool is_headland_turn_complete() const;
    nav_msgs::msg::Path generate_headland_turn_path() const;
    nav_msgs::msg::Path generate_reacquire_prediction_path() const;
    nav_msgs::msg::Path blend_reacquire_paths(
        const nav_msgs::msg::Path &predicted_path,
        const nav_msgs::msg::Path &measured_path,
        double measured_weight) const;
    double compute_reacquire_measured_weight() const;
    double reacquire_distance() const;
    void publish_headland_turn_path();
    void publish_reacquire_reference_path(const nav_msgs::msg::Path &path);
    void publish_navigation_mode();
    std::string navigation_mode_name() const;

    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr center_line_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr confidence_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr safety_margin_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr headland_detected_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr headland_turn_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reacquire_reference_path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_mode_pub_;

    // 控制器参数
    double target_distance_;        // 目标跟随距离（距离路径前方多少米）
    double max_linear_speed_;       // 最大线速度
    double min_linear_speed_;       // 最小线速度
    double max_angular_speed_;      // 最大角速度
    double confidence_high_threshold_;
    double confidence_low_threshold_;
    double confidence_stop_threshold_;
    double confidence_min_speed_factor_;
    double safety_margin_high_;
    double safety_margin_mid_;
    double safety_margin_stop_;
    int max_low_confidence_frames_;
    bool use_quality_aware_control_;
    bool enable_headland_turn_;
    double headland_min_follow_distance_;
    double headland_row_spacing_;
    double headland_turn_radius_;
    double headland_exit_distance_;
    double headland_settle_distance_;
    double headland_path_step_;
    double headland_turn_forward_extension_;
    bool headland_use_continuous_curvature_turn_;
    bool headland_turn_use_safety_margin_speed_;
    double headland_turn_goal_tolerance_;
    double headland_turn_heading_tolerance_;
    double headland_reacquire_confidence_;
    double headland_reacquire_track_confidence_;
    double headland_reacquire_search_speed_;
    double headland_reacquire_search_angular_speed_;
    double headland_reacquire_max_distance_;
    double headland_reacquire_max_time_;
    double headland_reacquire_prediction_length_;
    int headland_reacquire_frames_;
    int max_headland_turns_;
    int headland_turn_direction_;
    double headland_turn_linear_speed_;
    double headland_turn_target_distance_;
    
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
    nav_msgs::msg::Path last_valid_center_line_;
    bool has_center_line_;             // 是否收到中心线
    bool has_last_valid_center_line_;
    bool use_recovery_path_;
    int low_confidence_count_;
    double corridor_confidence_;
    double corridor_safety_margin_;
    bool has_corridor_confidence_;
    bool has_corridor_safety_margin_;
    NavigationMode navigation_mode_;
    nav_msgs::msg::Path headland_turn_path_;
    nav_msgs::msg::Path predicted_reacquire_path_;
    nav_msgs::msg::Path measured_reacquire_path_;
    double distance_since_last_turn_;
    double reacquire_start_x_;
    double reacquire_start_y_;
    double previous_odom_x_;
    double previous_odom_y_;
    bool has_previous_odom_;
    int completed_headland_turns_;
    int reacquire_count_;
    double headland_turn_goal_yaw_;
    bool has_headland_detected_;
    bool headland_detected_;
    bool has_predicted_reacquire_path_;
    bool has_measured_reacquire_path_;
    bool reacquire_failed_;
    rclcpp::Time reacquire_start_time_;
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
