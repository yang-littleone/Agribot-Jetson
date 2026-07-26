#include "centerline_extraction/pid_controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

PIDController::PIDController() : Node("pid_controller"), has_center_line_(false), first_run_(true)
{
    // 订阅玉米行中心线（路径）
    center_line_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/corn_row_center_line", 10,
        std::bind(&PIDController::center_line_callback, this, std::placeholders::_1));

    // 订阅里程计信息（小车位姿）
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&PIDController::odom_callback, this, std::placeholders::_1));

    confidence_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/corridor_confidence", 10,
        std::bind(&PIDController::confidence_callback, this, std::placeholders::_1));

    safety_margin_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/corridor_safety_margin", 10,
        std::bind(&PIDController::safety_margin_callback, this, std::placeholders::_1));
    headland_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/headland_detected", 10,
        std::bind(&PIDController::headland_detected_callback, this, std::placeholders::_1));

    // 发布速度控制指令
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    // 发布目标点可视化标记（调试用）
    target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/target_point", 10);
    headland_turn_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/headland_turn_path", 10);
    reacquire_reference_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/reacquire_reference_path", 10);
    navigation_mode_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/navigation_mode", 10);
    control_state_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/navigation_safety_state", 10);
    quality_factor_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/control_quality_factor", 10);
    lateral_error_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/controller_lateral_error", 10);
    heading_error_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/controller_heading_error", 10);

    // 声明并初始化参数
    this->declare_parameter("target_distance", 0.5);   // 目标跟随距离（米）
    this->declare_parameter("max_linear_speed", 0.4);  // 最大线速度（米/秒）
    this->declare_parameter("min_linear_speed", 0.1);  // 最小线速度（米/秒）
    this->declare_parameter("max_angular_speed", 0.35); // 最大角速度（弧度/秒）
    this->declare_parameter("max_angular_acceleration", 0.8); // 最大角加速度（弧度/秒方）
    this->declare_parameter("angular_command_deadband", 0.02); // 小角速度死区（弧度/秒）
    // 默认关闭：玉米行检测输出的是不断更新的局部路径，不能据其末点停车。
    // PID 室内测试启动文件会显式打开该项。
    this->declare_parameter("stop_at_path_end", false);
    this->declare_parameter("path_end_tolerance", 0.12);
    this->declare_parameter("confidence_high_threshold", 0.75);
    this->declare_parameter("confidence_low_threshold", 0.45);
    this->declare_parameter("confidence_stop_threshold", 0.25);
    this->declare_parameter("confidence_min_speed_factor", 0.2);
    this->declare_parameter("safety_margin_high", 0.20);
    this->declare_parameter("safety_margin_mid", 0.10);
    this->declare_parameter("safety_margin_stop", 0.05);
    this->declare_parameter("max_low_confidence_frames", 10);
    this->declare_parameter("max_low_confidence_duration", 0.50);
    this->declare_parameter("use_quality_aware_control", true);
    this->declare_parameter("require_quality_metrics", false);
    this->declare_parameter("centerline_timeout", 0.50);
    this->declare_parameter("quality_timeout", 0.50);
    this->declare_parameter("max_row_follow_distance", 0.0);
    this->declare_parameter("max_row_follow_time", 0.0);
    this->declare_parameter("enable_headland_turn", false);
    this->declare_parameter("headland_min_follow_distance", 1.5);
    this->declare_parameter("headland_row_spacing", 1.00); // 行间距，默认1.00米
    this->declare_parameter("headland_turn_radius", 0.0);
    this->declare_parameter("headland_exit_distance", 0.15); // 转弯后退出距离，默认0.15米
    this->declare_parameter("headland_settle_distance", 0.6);
    this->declare_parameter("headland_path_step", 0.08);
    this->declare_parameter("headland_turn_forward_extension", 0.40);        // 转弯前向延伸距离，默认0.25米
    this->declare_parameter("headland_use_continuous_curvature_turn", true); // 是否使用连续曲率转弯，默认true
    this->declare_parameter("headland_turn_use_safety_margin_speed", true);
    this->declare_parameter("headland_turn_goal_tolerance", 0.25);
    this->declare_parameter("headland_turn_heading_tolerance", 0.6);
    this->declare_parameter("headland_reacquire_confidence", 0.75);
    this->declare_parameter("headland_reacquire_track_confidence", 0.35);
    this->declare_parameter("headland_reacquire_search_speed", 0.08);
    this->declare_parameter("headland_reacquire_search_angular_speed", 0.0);
    this->declare_parameter("headland_reacquire_max_distance", 1.5);
    this->declare_parameter("headland_reacquire_max_time", 8.0);
    this->declare_parameter("headland_reacquire_prediction_length", 1.5);
    this->declare_parameter("headland_reacquire_frames", 5);
    this->declare_parameter("max_headland_turns", 0);
    this->declare_parameter("headland_turn_direction", "left");
    this->declare_parameter("headland_turn_linear_speed", 0.22);
    this->declare_parameter("headland_turn_target_distance", 0.35);

    // 横向PID参数
    this->declare_parameter("lateral_kp", 0.6); // 比例系数
    this->declare_parameter("lateral_ki", 0.0);  // 积分系数
    this->declare_parameter("lateral_kd", 0.0); // 微分系数

    // 航向PID参数
    this->declare_parameter("heading_kp", 0.7); // 比例系数
    this->declare_parameter("heading_ki", 0.0);  // 积分系数
    this->declare_parameter("heading_kd", 0.0); // 微分系数

    this->declare_parameter("debug_mode", false); // 调试模式开关

    // 读取参数
    get_parameters();

    // 初始化状态变量
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_yaw_ = 0.0;
    current_linear_vel_ = 0.0;
    current_angular_vel_ = 0.0;
    has_last_valid_center_line_ = false;
    use_recovery_path_ = false;
    low_confidence_count_ = 0;
    low_confidence_active_ = false;
    corridor_confidence_ = 1.0;
    corridor_safety_margin_ = safety_margin_high_;
    has_corridor_confidence_ = false;
    has_corridor_safety_margin_ = false;
    navigation_mode_ = NavigationMode::ROW_FOLLOW;
    distance_since_last_turn_ = 0.0;
    previous_odom_x_ = 0.0;
    previous_odom_y_ = 0.0;
    reacquire_start_x_ = 0.0;
    reacquire_start_y_ = 0.0;
    has_previous_odom_ = false;
    completed_headland_turns_ = 0;
    reacquire_count_ = 0;
    headland_turn_goal_yaw_ = 0.0;
    has_headland_detected_ = false;
    headland_detected_ = false;
    has_predicted_reacquire_path_ = false;
    has_measured_reacquire_path_ = false;
    reacquire_failed_ = false;
    reacquire_start_time_ = this->now();
    last_centerline_time_ = this->now();
    last_confidence_time_ = this->now();
    last_safety_margin_time_ = this->now();
    low_confidence_start_time_ = this->now();
    trial_start_time_ = this->now();
    has_centerline_timestamp_ = false;
    has_confidence_timestamp_ = false;
    has_safety_margin_timestamp_ = false;
    trial_started_ = false;
    trial_distance_ = 0.0;
    trial_previous_x_ = 0.0;
    trial_previous_y_ = 0.0;
    has_trial_previous_odom_ = false;
    last_control_state_.clear();

    // 初始化PID控制器状态变量
    lateral_integral_ = 0.0;
    lateral_previous_error_ = 0.0;
    heading_integral_ = 0.0;
    heading_previous_error_ = 0.0;
    target_path_heading_ = 0.0;
    last_angular_command_ = 0.0;
    has_last_angular_command_ = true;

    RCLCPP_INFO(this->get_logger(), "PID控制器初始化完成（目标距离: %.2f米）", target_distance_);
    RCLCPP_INFO(this->get_logger(), "置信度/安全裕度控制: %s",
                use_quality_aware_control_ ? "启用" : "关闭");
    RCLCPP_INFO(this->get_logger(), "地头U形换行: %s",
                enable_headland_turn_ ? "启用" : "关闭");
    publish_navigation_mode();
    publish_control_state("WAITING_FOR_CENTERLINE");
    publish_quality_factor(0.0);
}

void PIDController::get_parameters()
{
    this->get_parameter("target_distance", target_distance_);
    this->get_parameter("max_linear_speed", max_linear_speed_);
    this->get_parameter("min_linear_speed", min_linear_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("max_angular_acceleration", max_angular_acceleration_);
    this->get_parameter("angular_command_deadband", angular_command_deadband_);
    max_angular_speed_ = std::max(0.05, max_angular_speed_);
    max_angular_acceleration_ = std::max(0.0, max_angular_acceleration_);
    angular_command_deadband_ = std::clamp(
        angular_command_deadband_, 0.0, max_angular_speed_);
    this->get_parameter("stop_at_path_end", stop_at_path_end_);
    this->get_parameter("path_end_tolerance", path_end_tolerance_);
    path_end_tolerance_ = std::max(0.02, path_end_tolerance_);
    this->get_parameter("confidence_high_threshold", confidence_high_threshold_);
    this->get_parameter("confidence_low_threshold", confidence_low_threshold_);
    this->get_parameter("confidence_stop_threshold", confidence_stop_threshold_);
    this->get_parameter("confidence_min_speed_factor", confidence_min_speed_factor_);
    this->get_parameter("safety_margin_high", safety_margin_high_);
    this->get_parameter("safety_margin_mid", safety_margin_mid_);
    this->get_parameter("safety_margin_stop", safety_margin_stop_);
    this->get_parameter("max_low_confidence_frames", max_low_confidence_frames_);
    this->get_parameter("max_low_confidence_duration", max_low_confidence_duration_);
    this->get_parameter("use_quality_aware_control", use_quality_aware_control_);
    this->get_parameter("require_quality_metrics", require_quality_metrics_);
    this->get_parameter("centerline_timeout", centerline_timeout_);
    this->get_parameter("quality_timeout", quality_timeout_);
    this->get_parameter("max_row_follow_distance", max_row_follow_distance_);
    this->get_parameter("max_row_follow_time", max_row_follow_time_);
    max_low_confidence_duration_ = std::max(0.0, max_low_confidence_duration_);
    centerline_timeout_ = std::max(0.05, centerline_timeout_);
    quality_timeout_ = std::max(0.05, quality_timeout_);
    max_row_follow_distance_ = std::max(0.0, max_row_follow_distance_);
    max_row_follow_time_ = std::max(0.0, max_row_follow_time_);
    this->get_parameter("enable_headland_turn", enable_headland_turn_);
    this->get_parameter("headland_min_follow_distance", headland_min_follow_distance_);
    this->get_parameter("headland_row_spacing", headland_row_spacing_);
    this->get_parameter("headland_turn_radius", headland_turn_radius_);
    this->get_parameter("headland_exit_distance", headland_exit_distance_);
    this->get_parameter("headland_settle_distance", headland_settle_distance_);
    this->get_parameter("headland_path_step", headland_path_step_);
    this->get_parameter("headland_turn_forward_extension", headland_turn_forward_extension_);
    this->get_parameter("headland_use_continuous_curvature_turn", headland_use_continuous_curvature_turn_);
    this->get_parameter("headland_turn_use_safety_margin_speed", headland_turn_use_safety_margin_speed_);
    this->get_parameter("headland_turn_goal_tolerance", headland_turn_goal_tolerance_);
    this->get_parameter("headland_turn_heading_tolerance", headland_turn_heading_tolerance_);
    this->get_parameter("headland_reacquire_confidence", headland_reacquire_confidence_);
    this->get_parameter("headland_reacquire_track_confidence", headland_reacquire_track_confidence_);
    this->get_parameter("headland_reacquire_search_speed", headland_reacquire_search_speed_);
    this->get_parameter("headland_reacquire_search_angular_speed", headland_reacquire_search_angular_speed_);
    this->get_parameter("headland_reacquire_max_distance", headland_reacquire_max_distance_);
    this->get_parameter("headland_reacquire_max_time", headland_reacquire_max_time_);
    this->get_parameter("headland_reacquire_prediction_length", headland_reacquire_prediction_length_);
    this->get_parameter("headland_reacquire_frames", headland_reacquire_frames_);
    this->get_parameter("max_headland_turns", max_headland_turns_);
    this->get_parameter("headland_turn_linear_speed", headland_turn_linear_speed_);
    this->get_parameter("headland_turn_target_distance", headland_turn_target_distance_);

    std::string turn_direction = "left";
    this->get_parameter("headland_turn_direction", turn_direction);
    headland_turn_direction_ = (turn_direction == "right") ? -1 : 1;
    if (headland_row_spacing_ <= 0.0)
    {
        headland_row_spacing_ = 0.87;
    }
    if (headland_turn_radius_ <= 0.0)
    {
        headland_turn_radius_ = headland_row_spacing_ * 0.5;
    }
    if (headland_path_step_ <= 0.0)
    {
        headland_path_step_ = 0.08;
    }
    headland_exit_distance_ = std::max(0.0, headland_exit_distance_);
    headland_settle_distance_ = std::max(0.0, headland_settle_distance_);
    if (headland_turn_forward_extension_ <= 0.0)
    {
        headland_turn_forward_extension_ = std::min(headland_turn_radius_, 0.25);
    }
    headland_turn_forward_extension_ = std::clamp(
        headland_turn_forward_extension_, 0.05, std::max(0.05, headland_turn_radius_));
    if (headland_reacquire_frames_ < 1)
    {
        headland_reacquire_frames_ = 1;
    }
    headland_turn_linear_speed_ = std::clamp(headland_turn_linear_speed_, 0.05, max_linear_speed_);
    headland_turn_target_distance_ = std::clamp(headland_turn_target_distance_, 0.1, 1.0);
    headland_reacquire_confidence_ = std::clamp(headland_reacquire_confidence_, 0.0, 1.0);
    headland_reacquire_track_confidence_ = std::clamp(
        headland_reacquire_track_confidence_, 0.0, headland_reacquire_confidence_);
    headland_reacquire_search_speed_ = std::clamp(
        headland_reacquire_search_speed_, 0.0, max_linear_speed_);
    headland_reacquire_search_angular_speed_ = std::clamp(
        headland_reacquire_search_angular_speed_, -max_angular_speed_, max_angular_speed_);
    if (headland_reacquire_max_distance_ <= 0.0)
    {
        headland_reacquire_max_distance_ = 1.5;
    }
    if (headland_reacquire_max_time_ <= 0.0)
    {
        headland_reacquire_max_time_ = 8.0;
    }
    headland_reacquire_prediction_length_ = std::max(
        headland_reacquire_prediction_length_, target_distance_ + headland_path_step_);

    this->get_parameter("lateral_kp", lateral_kp_);
    this->get_parameter("lateral_ki", lateral_ki_);
    this->get_parameter("lateral_kd", lateral_kd_);

    this->get_parameter("heading_kp", heading_kp_);
    this->get_parameter("heading_ki", heading_ki_);
    this->get_parameter("heading_kd", heading_kd_);

    this->get_parameter("debug_mode", debug_mode_);
}

void PIDController::center_line_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    last_centerline_time_ = this->now();
    has_centerline_timestamp_ = true;
    if (navigation_mode_ == NavigationMode::U_TURN)
    {
        return;
    }

    if (msg->poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "收到空的中心线，当前帧不更新中心线");
        if (navigation_mode_ == NavigationMode::NEXT_ROW_REACQUIRE &&
            has_predicted_reacquire_path_)
        {
            center_line_ = predicted_reacquire_path_;
            has_center_line_ = true;
            has_measured_reacquire_path_ = false;
        }
        else
        {
            has_center_line_ = false;
        }
        return;
    }

    if (navigation_mode_ == NavigationMode::NEXT_ROW_REACQUIRE)
    {
        measured_reacquire_path_ = *msg;
        has_measured_reacquire_path_ = true;
        if (has_predicted_reacquire_path_)
        {
            center_line_ = blend_reacquire_paths(
                predicted_reacquire_path_,
                measured_reacquire_path_,
                compute_reacquire_measured_weight());
        }
        else
        {
            center_line_ = measured_reacquire_path_;
        }
    }
    else
    {
        center_line_ = *msg;
    }
    has_center_line_ = true;
    if (!trial_started_ && navigation_mode_ == NavigationMode::ROW_FOLLOW)
    {
        trial_started_ = true;
        trial_start_time_ = this->now();
        trial_distance_ = 0.0;
        has_trial_previous_odom_ = false;
        RCLCPP_INFO(this->get_logger(), "田间试验计程开始");
    }
    if (!use_quality_aware_control_ ||
        !has_corridor_confidence_ ||
        corridor_confidence_ >= confidence_low_threshold_)
    {
        last_valid_center_line_ = center_line_;
        has_last_valid_center_line_ = true;
        low_confidence_count_ = 0;
    }
    RCLCPP_DEBUG(this->get_logger(), "收到中心线（%zu个点）", center_line_.poses.size());
}

void PIDController::confidence_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    corridor_confidence_ = std::clamp(static_cast<double>(msg->data), 0.0, 1.0);
    has_corridor_confidence_ = true;
    last_confidence_time_ = this->now();
    has_confidence_timestamp_ = true;
}

void PIDController::safety_margin_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    corridor_safety_margin_ = static_cast<double>(msg->data);
    has_corridor_safety_margin_ = true;
    last_safety_margin_time_ = this->now();
    has_safety_margin_timestamp_ = true;
}

void PIDController::headland_detected_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    has_headland_detected_ = true;
    headland_detected_ = msg->data;
}

void PIDController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 更新当前位姿
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_linear_vel_ = msg->twist.twist.linear.x;
    current_angular_vel_ = msg->twist.twist.angular.z;

    // 从四元数解析偏航角（yaw）
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
    update_travel_distance(current_x_, current_y_);
    publish_navigation_mode();

    if (enable_headland_turn_ &&
        navigation_mode_ == NavigationMode::ROW_FOLLOW &&
        should_start_headland_turn())
    {
        start_headland_turn();
    }

    if (navigation_mode_ == NavigationMode::U_TURN)
    {
        publish_headland_turn_path();
        if (is_headland_turn_complete())
        {
            start_next_row_reacquire();
            publish_stop_command();
            return;
        }

        if (has_center_line_ && !center_line_.poses.empty())
        {
            auto cmd_vel = calculate_control_command();
            cmd_vel_pub_->publish(cmd_vel);
        }
        return;
    }

    if (navigation_mode_ == NavigationMode::NEXT_ROW_REACQUIRE)
    {
        const bool reacquired =
            has_measured_reacquire_path_ &&
            !measured_reacquire_path_.poses.empty() &&
            has_corridor_confidence_ &&
            corridor_confidence_ >= headland_reacquire_confidence_;
        reacquire_count_ = reacquired ? reacquire_count_ + 1 : 0;
        if (reacquire_count_ >= headland_reacquire_frames_)
        {
            navigation_mode_ = NavigationMode::ROW_FOLLOW;
            distance_since_last_turn_ = 0.0;
            reacquire_count_ = 0;
            if (has_measured_reacquire_path_)
            {
                center_line_ = measured_reacquire_path_;
                last_valid_center_line_ = measured_reacquire_path_;
                has_center_line_ = true;
                has_last_valid_center_line_ = true;
            }
            predicted_reacquire_path_.poses.clear();
            measured_reacquire_path_.poses.clear();
            has_predicted_reacquire_path_ = false;
            has_measured_reacquire_path_ = false;
            reacquire_failed_ = false;
            reset_pid_state();
            RCLCPP_INFO(this->get_logger(), "相邻行中心线重捕获完成，恢复行间跟踪");
        }
        else
        {
            const double search_distance = reacquire_distance();
            const double search_time = std::max(0.0, (this->now() - reacquire_start_time_).seconds());
            if (!reacquire_failed_ &&
                (search_distance >= headland_reacquire_max_distance_ ||
                 search_time >= headland_reacquire_max_time_))
            {
                reacquire_failed_ = true;
                RCLCPP_ERROR(this->get_logger(),
                             "相邻行中心线重捕获失败，停车: distance=%.2f/%.2f m, time=%.2f/%.2f s",
                             search_distance, headland_reacquire_max_distance_,
                             search_time, headland_reacquire_max_time_);
            }

            if (reacquire_failed_)
            {
                publish_stop_command();
                return;
            }

            const bool can_track_measured =
                has_measured_reacquire_path_ &&
                !measured_reacquire_path_.poses.empty() &&
                (!has_corridor_confidence_ ||
                 corridor_confidence_ >= headland_reacquire_track_confidence_);
            const double measured_weight = compute_reacquire_measured_weight();

            geometry_msgs::msg::Twist cmd_vel;
            if (can_track_measured && has_predicted_reacquire_path_)
            {
                center_line_ = blend_reacquire_paths(
                    predicted_reacquire_path_,
                    measured_reacquire_path_,
                    measured_weight);
                has_center_line_ = !center_line_.poses.empty();
            }
            else if (can_track_measured)
            {
                center_line_ = measured_reacquire_path_;
                has_center_line_ = true;
            }
            else if (has_predicted_reacquire_path_)
            {
                center_line_ = predicted_reacquire_path_;
                has_center_line_ = true;
            }

            if (has_center_line_ && !center_line_.poses.empty())
            {
                publish_reacquire_reference_path(center_line_);
                cmd_vel = calculate_control_command();
                cmd_vel.linear.x = std::clamp(
                    cmd_vel.linear.x, 0.0, headland_reacquire_search_speed_);
            }
            else
            {
                cmd_vel.linear.x = headland_reacquire_search_speed_;
                cmd_vel.angular.z = headland_reacquire_search_angular_speed_;
            }
            cmd_vel_pub_->publish(cmd_vel);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "重获相邻行: confidence=%.2f, stable_frames=%d/%d, measured=%s, blend=%.2f, distance=%.2f/%.2f, time=%.2f/%.2f",
                                 corridor_confidence_, reacquire_count_, headland_reacquire_frames_,
                                 can_track_measured ? "true" : "false", measured_weight,
                                 search_distance, headland_reacquire_max_distance_,
                                 search_time, headland_reacquire_max_time_);
            return;
        }
    }

    if (trial_limit_reached())
    {
        publish_stop_command();
        publish_quality_factor(0.0);
        const double elapsed = trial_started_
                                   ? std::max(0.0, (this->now() - trial_start_time_).seconds())
                                   : 0.0;
        const bool distance_reached =
            max_row_follow_distance_ > 0.0 && trial_distance_ >= max_row_follow_distance_;
        publish_control_state(distance_reached ? "STOP_DISTANCE_LIMIT" : "STOP_TIME_LIMIT");
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "试验上限已到，停车: distance=%.2f/%.2f m, time=%.2f/%.2f s",
            trial_distance_, max_row_follow_distance_, elapsed, max_row_follow_time_);
        return;
    }

    if (!has_center_line_ || center_line_.poses.empty())
    {
        publish_stop_command();
        publish_quality_factor(0.0);
        publish_control_state("WAITING_FOR_CENTERLINE");
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "未收到有效中心线，发布停止指令");
        return;
    }

    if (centerline_is_stale())
    {
        publish_stop_command();
        publish_quality_factor(0.0);
        publish_control_state("STOP_STALE_CENTERLINE");
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "中心线超过 %.2f s 未更新，停车", centerline_timeout_);
        return;
    }

    if (use_quality_aware_control_ && require_quality_metrics_ &&
        (!has_corridor_confidence_ || !has_corridor_safety_margin_))
    {
        publish_stop_command();
        publish_quality_factor(0.0);
        publish_control_state("WAITING_FOR_QUALITY");
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "等待置信度和安全裕度，停车");
        return;
    }

    if (quality_data_is_stale())
    {
        publish_stop_command();
        publish_quality_factor(0.0);
        publish_control_state("STOP_STALE_QUALITY");
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "质量指标超过 %.2f s 未更新，停车", quality_timeout_);
        return;
    }

    if (should_stop_for_safety())
    {
        publish_stop_command();
        publish_quality_factor(0.0);
        publish_control_state("STOP_SAFETY_MARGIN");
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "安全裕度不足 %.3f m，停车", corridor_safety_margin_);
        return;
    }

    const bool confidence_low = use_quality_aware_control_ &&
                                has_corridor_confidence_ &&
                                corridor_confidence_ < confidence_low_threshold_;
    use_recovery_path_ = false;

    if (confidence_low)
    {
        low_confidence_count_++;
        if (!low_confidence_active_)
        {
            low_confidence_active_ = true;
            low_confidence_start_time_ = this->now();
        }
        const double low_confidence_duration =
            std::max(0.0, (this->now() - low_confidence_start_time_).seconds());
        if (has_last_valid_center_line_ &&
            low_confidence_duration <= max_low_confidence_duration_ &&
            corridor_confidence_ >= confidence_stop_threshold_)
        {
            center_line_ = last_valid_center_line_;
            has_center_line_ = true;
            use_recovery_path_ = true;
            publish_control_state("RECOVERY_HISTORY_PATH");
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "低置信度 %.2f，使用历史中心线 %.2f/%.2f s",
                                 corridor_confidence_, low_confidence_duration,
                                 max_low_confidence_duration_);
        }
        else
        {
            publish_stop_command();
            publish_quality_factor(0.0);
            publish_control_state("STOP_LOW_CONFIDENCE");
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "中心线置信度过低 %.2f，停车", corridor_confidence_);
            return;
        }
    }
    else
    {
        low_confidence_count_ = 0;
        low_confidence_active_ = false;
    }

    if (stop_at_path_end_ && is_path_end_reached())
    {
        publish_stop_command();
        publish_quality_factor(0.0);
        publish_control_state("STOP_PATH_END");
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "已到达测试路径终点（容差 %.2f m），停车", path_end_tolerance_);
    }
    else if (has_center_line_ && !center_line_.poses.empty())
    {
        auto cmd_vel = calculate_control_command();
        cmd_vel_pub_->publish(cmd_vel);
        if (!use_recovery_path_)
        {
            publish_control_state("TRACKING");
        }
    }
    else
    {
        // 当没有中心线时，发布零速度指令让小车停止
        publish_stop_command();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "未收到中心线，发布停止指令");
    }
}

bool PIDController::is_path_end_reached() const
{
    if (!has_center_line_ || center_line_.poses.empty())
    {
        return false;
    }

    const auto &end = center_line_.poses.back().pose.position;
    const double end_distance = std::hypot(end.x - current_x_, end.y - current_y_);
    if (end_distance <= path_end_tolerance_)
    {
        return true;
    }

    // 若车辆因横向偏差没有进入终点圆形容差，却已经越过终点横线，也必须停车。
    // 否则 find_target_point() 会持续把最后一个点当目标，小车会在终点外继续前进。
    if (center_line_.poses.size() < 2)
    {
        return false;
    }

    // U 形/半圆路径的终点朝向可能与起点相反。若仅检查终点横线，起点会被
    // 误判为已经越过终点。因此只有最近路径点已位于最后约 0.10 m 时，才允许
    // 使用“越过终点横线”停车保护。
    size_t closest_idx = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (size_t index = 0; index < center_line_.poses.size(); ++index)
    {
        const auto &point = center_line_.poses[index].pose.position;
        const double distance = std::hypot(point.x - current_x_, point.y - current_y_);
        if (distance < closest_distance)
        {
            closest_distance = distance;
            closest_idx = index;
        }
    }
    constexpr size_t kEndPointGateSamples = 3;
    if (closest_idx + kEndPointGateSamples < center_line_.poses.size())
    {
        return false;
    }

    const auto &previous = center_line_.poses[center_line_.poses.size() - 2].pose.position;
    const double tangent_x = end.x - previous.x;
    const double tangent_y = end.y - previous.y;
    const double tangent_length = std::hypot(tangent_x, tangent_y);
    if (tangent_length < 1e-6)
    {
        return false;
    }

    const double passed_distance =
        ((current_x_ - end.x) * tangent_x + (current_y_ - end.y) * tangent_y) / tangent_length;
    return passed_distance >= 0.0;
}

geometry_msgs::msg::PointStamped PIDController::find_target_point()
{
    geometry_msgs::msg::PointStamped target_point;
    target_point.header.frame_id = center_line_.header.frame_id;
    target_point.header.stamp = this->now();
    target_path_heading_ = current_yaw_;
    const double active_target_distance =
        (navigation_mode_ == NavigationMode::U_TURN) ? headland_turn_target_distance_ : target_distance_;

    if (center_line_.poses.size() == 1)
    {
        target_point.point = center_line_.poses.front().pose.position;
        return target_point;
    }

    // 1. 找到路径上离小车最近的点
    size_t closest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < center_line_.poses.size(); ++i)
    {
        const auto &p = center_line_.poses[i].pose.position;
        double dx = p.x - current_x_;
        double dy = p.y - current_y_;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // 2. 从最近点开始，寻找距离小车目标距离的点
    double accumulated_dist = 0.0;
    for (size_t i = closest_idx; i < center_line_.poses.size() - 1; ++i)
    {
        const auto &p1 = center_line_.poses[i].pose.position;
        const auto &p2 = center_line_.poses[i + 1].pose.position;

        // 计算路径段的长度
        double segment_length = std::hypot(p2.x - p1.x, p2.y - p1.y);

        // 如果加上这一段超过了目标距离，则在该段上插值
        if (accumulated_dist + segment_length >= active_target_distance)
        {
            double remaining_dist = active_target_distance - accumulated_dist;
            double t = remaining_dist / segment_length;

            target_point.point.x = p1.x + t * (p2.x - p1.x);
            target_point.point.y = p1.y + t * (p2.y - p1.y);
            target_point.point.z = 0.0;
            target_path_heading_ = std::atan2(p2.y - p1.y, p2.x - p1.x);

            return target_point;
        }

        accumulated_dist += segment_length;
    }

    auto last_point = center_line_.poses.back().pose.position;
    if (center_line_.poses.size() >= 2)
    {
        const auto &previous_point =
            center_line_.poses[center_line_.poses.size() - 2].pose.position;
        target_path_heading_ = std::atan2(
            last_point.y - previous_point.y,
            last_point.x - previous_point.x);
    }
    const double dx = last_point.x - current_x_;
    const double dy = last_point.y - current_y_;
    const double x_veh = dx * std::cos(current_yaw_) + dy * std::sin(current_yaw_);
    if (navigation_mode_ == NavigationMode::ROW_FOLLOW && x_veh < 0.0)
    {
        target_point.point.x = current_x_ + active_target_distance * std::cos(current_yaw_);
        target_point.point.y = current_y_ + active_target_distance * std::sin(current_yaw_);
        target_point.point.z = 0.0;
        target_path_heading_ = current_yaw_;
    }
    else
    {
        target_point.point = last_point;
    }

    return target_point;
}

double PIDController::compute_lateral_error_to_path() const
{
    if (center_line_.poses.empty())
    {
        return 0.0;
    }
    if (center_line_.poses.size() == 1)
    {
        const auto &point = center_line_.poses.front().pose.position;
        const double dx = point.x - current_x_;
        const double dy = point.y - current_y_;
        return dx * -std::sin(target_path_heading_) +
               dy * std::cos(target_path_heading_);
    }

    double best_distance_squared = std::numeric_limits<double>::infinity();
    double best_lateral_error = 0.0;
    for (size_t index = 0; index + 1 < center_line_.poses.size(); ++index)
    {
        const auto &start = center_line_.poses[index].pose.position;
        const auto &end = center_line_.poses[index + 1].pose.position;
        const double segment_x = end.x - start.x;
        const double segment_y = end.y - start.y;
        const double segment_length_squared =
            segment_x * segment_x + segment_y * segment_y;
        if (segment_length_squared < 1e-12)
        {
            continue;
        }

        const double projection_fraction = std::clamp(
            ((current_x_ - start.x) * segment_x +
             (current_y_ - start.y) * segment_y) /
                segment_length_squared,
            0.0,
            1.0);
        const double projection_x =
            start.x + projection_fraction * segment_x;
        const double projection_y =
            start.y + projection_fraction * segment_y;
        const double path_from_vehicle_x = projection_x - current_x_;
        const double path_from_vehicle_y = projection_y - current_y_;
        const double distance_squared =
            path_from_vehicle_x * path_from_vehicle_x +
            path_from_vehicle_y * path_from_vehicle_y;
        if (distance_squared >= best_distance_squared)
        {
            continue;
        }

        const double inverse_segment_length =
            1.0 / std::sqrt(segment_length_squared);
        const double left_normal_x =
            -segment_y * inverse_segment_length;
        const double left_normal_y =
            segment_x * inverse_segment_length;
        best_distance_squared = distance_squared;
        best_lateral_error =
            path_from_vehicle_x * left_normal_x +
            path_from_vehicle_y * left_normal_y;
    }
    return best_lateral_error;
}

double PIDController::limit_angular_command(
    double desired_angular_speed, double angular_speed_limit, double dt)
{
    desired_angular_speed = std::clamp(
        desired_angular_speed, -angular_speed_limit, angular_speed_limit);
    if (std::abs(desired_angular_speed) < angular_command_deadband_)
    {
        desired_angular_speed = 0.0;
    }

    if (!has_last_angular_command_)
    {
        last_angular_command_ = 0.0;
        has_last_angular_command_ = true;
    }
    if (max_angular_acceleration_ > 0.0)
    {
        const double max_delta =
            max_angular_acceleration_ * std::clamp(dt, 0.001, 0.2);
        desired_angular_speed = std::clamp(
            desired_angular_speed,
            last_angular_command_ - max_delta,
            last_angular_command_ + max_delta);
    }
    // 死区只作用于 PID 的目标指令，不能再次作用于角加速度限幅后的中间值。
    // 否则在高频控制下，当 max_angular_acceleration * dt 小于死区时，
    // 每一帧刚建立的角速度都会被清零，车辆将永远无法开始转向。
    desired_angular_speed = std::clamp(
        desired_angular_speed, -angular_speed_limit, angular_speed_limit);
    last_angular_command_ = desired_angular_speed;
    return desired_angular_speed;
}

void PIDController::publish_stop_command()
{
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    reset_pid_state();
}

double PIDController::compute_pid(double error, double dt, double &integral, double &previous_error,
                                  double kp, double ki, double kd)
{
    // 防止除零错误和时间间隔过大
    if (dt <= 0.0 || dt > 1.0)
    {
        dt = 0.05;
    }

    // 积分项
    integral += error * dt;

    // 微分项
    double derivative = (error - previous_error) / dt;

    // PID输出
    double output = kp * error + ki * integral + kd * derivative;

    // 更新previous_error
    previous_error = error;

    return output;
}

double PIDController::compute_confidence_factor() const
{
    if (navigation_mode_ == NavigationMode::U_TURN)
    {
        return 1.0;
    }

    if (navigation_mode_ == NavigationMode::NEXT_ROW_REACQUIRE)
    {
        return 1.0;
    }

    if (!use_quality_aware_control_)
    {
        return 1.0;
    }

    if (!has_corridor_confidence_)
    {
        return 1.0;
    }

    if (use_recovery_path_)
    {
        return confidence_min_speed_factor_;
    }

    return std::clamp(corridor_confidence_, confidence_min_speed_factor_, 1.0);
}

double PIDController::compute_safety_factor() const
{
    if (navigation_mode_ == NavigationMode::U_TURN)
    {
        if (!use_quality_aware_control_ ||
            !headland_turn_use_safety_margin_speed_ ||
            !has_corridor_safety_margin_)
        {
            return 1.0;
        }
        if (corridor_safety_margin_ < safety_margin_stop_)
        {
            return 0.35;
        }
        if (corridor_safety_margin_ < safety_margin_mid_)
        {
            return 0.5;
        }
        if (corridor_safety_margin_ < safety_margin_high_)
        {
            return 0.75;
        }
        return 1.0;
    }

    if (navigation_mode_ == NavigationMode::NEXT_ROW_REACQUIRE)
    {
        return 1.0;
    }

    if (!use_quality_aware_control_)
    {
        return 1.0;
    }

    if (!has_corridor_safety_margin_)
    {
        return 1.0;
    }

    if (corridor_safety_margin_ < safety_margin_stop_)
    {
        return 0.0;
    }
    if (corridor_safety_margin_ < safety_margin_mid_)
    {
        return 0.3;
    }
    if (corridor_safety_margin_ < safety_margin_high_)
    {
        return 0.6;
    }
    return 1.0;
}

bool PIDController::should_stop_for_safety() const
{
    if (navigation_mode_ == NavigationMode::U_TURN ||
        navigation_mode_ == NavigationMode::NEXT_ROW_REACQUIRE)
    {
        return false;
    }

    return use_quality_aware_control_ &&
           has_corridor_safety_margin_ &&
           corridor_safety_margin_ < safety_margin_stop_;
}

bool PIDController::quality_data_is_stale() const
{
    if (!use_quality_aware_control_ || !require_quality_metrics_)
    {
        return false;
    }
    if (!has_confidence_timestamp_ || !has_safety_margin_timestamp_)
    {
        return true;
    }
    const rclcpp::Time now = this->now();
    return (now - last_confidence_time_).seconds() > quality_timeout_ ||
           (now - last_safety_margin_time_).seconds() > quality_timeout_;
}

bool PIDController::centerline_is_stale() const
{
    if (!has_centerline_timestamp_)
    {
        return true;
    }
    return (this->now() - last_centerline_time_).seconds() > centerline_timeout_;
}

bool PIDController::trial_limit_reached() const
{
    if (!trial_started_ || navigation_mode_ != NavigationMode::ROW_FOLLOW)
    {
        return false;
    }
    const bool distance_reached =
        max_row_follow_distance_ > 0.0 && trial_distance_ >= max_row_follow_distance_;
    const bool time_reached =
        max_row_follow_time_ > 0.0 &&
        (this->now() - trial_start_time_).seconds() >= max_row_follow_time_;
    return distance_reached || time_reached;
}

double PIDController::normalize_angle(double angle) const
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

void PIDController::reset_pid_state()
{
    lateral_integral_ = 0.0;
    lateral_previous_error_ = 0.0;
    heading_integral_ = 0.0;
    heading_previous_error_ = 0.0;
    last_angular_command_ = 0.0;
    has_last_angular_command_ = true;
    first_run_ = true;
}

void PIDController::update_travel_distance(double x, double y)
{
    if (!has_previous_odom_)
    {
        previous_odom_x_ = x;
        previous_odom_y_ = y;
        has_previous_odom_ = true;
        return;
    }

    const double step = std::hypot(x - previous_odom_x_, y - previous_odom_y_);
    if (step < 1.0)
    {
        distance_since_last_turn_ += step;
    }
    previous_odom_x_ = x;
    previous_odom_y_ = y;

    if (trial_started_ && navigation_mode_ == NavigationMode::ROW_FOLLOW)
    {
        if (!has_trial_previous_odom_)
        {
            trial_previous_x_ = x;
            trial_previous_y_ = y;
            has_trial_previous_odom_ = true;
        }
        else
        {
            const double trial_step =
                std::hypot(x - trial_previous_x_, y - trial_previous_y_);
            if (trial_step < 1.0)
            {
                trial_distance_ += trial_step;
            }
            trial_previous_x_ = x;
            trial_previous_y_ = y;
        }
    }
}

bool PIDController::should_start_headland_turn() const
{
    const bool turn_limit_ok =
        max_headland_turns_ <= 0 || completed_headland_turns_ < max_headland_turns_;
    if (!turn_limit_ok)
    {
        return false;
    }

    return has_headland_detected_ &&
           headland_detected_ &&
           distance_since_last_turn_ >= headland_min_follow_distance_;
}

void PIDController::start_headland_turn()
{
    const int active_turn_direction = headland_turn_direction_;
    headland_turn_goal_yaw_ = normalize_angle(
        current_yaw_ + static_cast<double>(active_turn_direction) * M_PI);
    headland_turn_path_ = generate_headland_turn_path();
    if (headland_turn_path_.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "无法生成地头U形路径，保持行间跟踪");
        return;
    }

    navigation_mode_ = NavigationMode::U_TURN;
    center_line_ = headland_turn_path_;
    has_center_line_ = true;
    completed_headland_turns_++;
    headland_detected_ = false;
    headland_turn_direction_ *= -1;
    reset_pid_state();
    publish_headland_turn_path();
    publish_navigation_mode();
    RCLCPP_WARN(this->get_logger(), "触发地头U形换行，当前第 %d 次，方向: %s，下一次方向: %s，路径点数: %zu",
                completed_headland_turns_,
                active_turn_direction > 0 ? "left" : "right",
                headland_turn_direction_ > 0 ? "left" : "right",
                headland_turn_path_.poses.size());
}

void PIDController::start_next_row_reacquire()
{
    navigation_mode_ = NavigationMode::NEXT_ROW_REACQUIRE;
    predicted_reacquire_path_ = generate_reacquire_prediction_path();
    measured_reacquire_path_.poses.clear();
    has_predicted_reacquire_path_ = !predicted_reacquire_path_.poses.empty();
    has_measured_reacquire_path_ = false;
    if (has_predicted_reacquire_path_)
    {
        center_line_ = predicted_reacquire_path_;
        has_center_line_ = true;
        publish_reacquire_reference_path(center_line_);
    }
    else
    {
        has_center_line_ = false;
    }
    has_last_valid_center_line_ = false;
    use_recovery_path_ = false;
    low_confidence_count_ = 0;
    reacquire_count_ = 0;
    reacquire_failed_ = false;
    reacquire_start_x_ = current_x_;
    reacquire_start_y_ = current_y_;
    reacquire_start_time_ = this->now();
    has_headland_detected_ = false;
    headland_detected_ = false;
    reset_pid_state();
    publish_navigation_mode();
    RCLCPP_WARN(this->get_logger(),
                "地头U形换行完成，跟踪预测中心线并重获相邻行: predicted_points=%zu, max_distance=%.2f m, max_time=%.2f s",
                predicted_reacquire_path_.poses.size(),
                headland_reacquire_max_distance_,
                headland_reacquire_max_time_);
}

bool PIDController::is_headland_turn_complete() const
{
    if (headland_turn_path_.poses.empty())
    {
        return false;
    }

    const auto &goal = headland_turn_path_.poses.back().pose.position;
    const double goal_dist = std::hypot(goal.x - current_x_, goal.y - current_y_);
    const double heading_error = std::abs(normalize_angle(current_yaw_ - headland_turn_goal_yaw_));
    return goal_dist <= headland_turn_goal_tolerance_ &&
           heading_error <= headland_turn_heading_tolerance_;
}

nav_msgs::msg::Path PIDController::generate_headland_turn_path() const
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = this->now();

    const double dir = static_cast<double>(headland_turn_direction_);
    const double radius = std::max(0.1, headland_turn_radius_);
    const double step = std::max(0.03, headland_path_step_);
    const double tx = std::cos(current_yaw_);
    const double ty = std::sin(current_yaw_);
    const double lx = -std::sin(current_yaw_);
    const double ly = std::cos(current_yaw_);

    auto push_pose = [&path](double x, double y, double yaw)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
        path.poses.push_back(pose);
    };

    for (double d = 0.0; d <= headland_exit_distance_ + 1e-6; d += step)
    {
        push_pose(current_x_ + tx * d, current_y_ + ty * d, current_yaw_);
    }

    const double sx = current_x_ + tx * headland_exit_distance_;
    const double sy = current_y_ + ty * headland_exit_distance_;

    double end_x = sx;
    double end_y = sy;
    const double end_yaw = normalize_angle(current_yaw_ + dir * M_PI);

    if (headland_use_continuous_curvature_turn_)
    {
        auto smoother_step = [](double u)
        {
            u = std::clamp(u, 0.0, 1.0);
            return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
        };

        auto smoother_step_derivative = [](double u)
        {
            u = std::clamp(u, 0.0, 1.0);
            return 30.0 * u * u * (1.0 - u) * (1.0 - u);
        };

        const double lateral_shift = 2.0 * radius;
        const double forward_extension = headland_turn_forward_extension_;
        const double estimated_length = M_PI * forward_extension + lateral_shift;
        const int samples = std::max(12, static_cast<int>(std::ceil(estimated_length / step)));

        for (int i = 1; i <= samples; ++i)
        {
            const double u = static_cast<double>(i) / static_cast<double>(samples);
            const double local_forward = forward_extension * std::sin(M_PI * u);
            const double local_lateral = lateral_shift * smoother_step(u);
            const double x = sx + tx * local_forward + dir * lx * local_lateral;
            const double y = sy + ty * local_forward + dir * ly * local_lateral;

            const double dx_du = forward_extension * M_PI * std::cos(M_PI * u);
            const double dy_du = lateral_shift * smoother_step_derivative(u);
            double yaw = end_yaw;
            if (i < samples)
            {
                yaw = normalize_angle(current_yaw_ + std::atan2(dir * dy_du, dx_du));
            }
            push_pose(x, y, yaw);
        }

        end_x = sx + 2.0 * dir * lx * radius;
        end_y = sy + 2.0 * dir * ly * radius;
    }
    else
    {
        const double cx = sx + dir * lx * radius;
        const double cy = sy + dir * ly * radius;
        const int samples = std::max(8, static_cast<int>(std::ceil(M_PI * radius / step)));
        for (int i = 1; i <= samples; ++i)
        {
            const double phi = M_PI * static_cast<double>(i) / static_cast<double>(samples);
            const double offset_x = -dir * lx * radius * std::cos(phi) + tx * radius * std::sin(phi);
            const double offset_y = -dir * ly * radius * std::cos(phi) + ty * radius * std::sin(phi);
            push_pose(cx + offset_x, cy + offset_y, normalize_angle(current_yaw_ + dir * phi));
        }

        end_x = sx + 2.0 * dir * lx * radius;
        end_y = sy + 2.0 * dir * ly * radius;
    }

    for (double d = step; d <= headland_settle_distance_ + 1e-6; d += step)
    {
        push_pose(end_x - tx * d, end_y - ty * d, end_yaw);
    }

    return path;
}

nav_msgs::msg::Path PIDController::generate_reacquire_prediction_path() const
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = this->now();

    const double step = std::max(0.03, headland_path_step_);
    const double length = std::max(
        headland_reacquire_prediction_length_,
        target_distance_ + step);
    const double tx = std::cos(current_yaw_);
    const double ty = std::sin(current_yaw_);

    for (double d = 0.0; d <= length + 1e-6; d += step)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = current_x_ + tx * d;
        pose.pose.position.y = current_y_ + ty * d;
        pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_yaw_);
        pose.pose.orientation = tf2::toMsg(q);
        path.poses.push_back(pose);
    }

    return path;
}

nav_msgs::msg::Path PIDController::blend_reacquire_paths(
    const nav_msgs::msg::Path &predicted_path,
    const nav_msgs::msg::Path &measured_path,
    double measured_weight) const
{
    if (predicted_path.poses.empty())
    {
        return measured_path;
    }
    if (measured_path.poses.empty())
    {
        return predicted_path;
    }
    if (!predicted_path.header.frame_id.empty() &&
        !measured_path.header.frame_id.empty() &&
        predicted_path.header.frame_id != measured_path.header.frame_id)
    {
        return measured_path;
    }

    const double alpha = std::clamp(measured_weight, 0.0, 1.0);
    if (alpha <= 1e-3)
    {
        return predicted_path;
    }
    if (alpha >= 1.0 - 1e-3)
    {
        return measured_path;
    }

    nav_msgs::msg::Path blended_path;
    blended_path.header = measured_path.header;
    if (blended_path.header.frame_id.empty())
    {
        blended_path.header = predicted_path.header;
    }
    blended_path.header.stamp = this->now();

    const size_t point_count = std::min(predicted_path.poses.size(), measured_path.poses.size());
    blended_path.poses.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i)
    {
        geometry_msgs::msg::PoseStamped pose = measured_path.poses[i];
        pose.header = blended_path.header;
        const auto &predicted = predicted_path.poses[i].pose.position;
        const auto &measured = measured_path.poses[i].pose.position;
        pose.pose.position.x = (1.0 - alpha) * predicted.x + alpha * measured.x;
        pose.pose.position.y = (1.0 - alpha) * predicted.y + alpha * measured.y;
        pose.pose.position.z = 0.0;
        blended_path.poses.push_back(pose);
    }

    for (size_t i = 0; i + 1 < blended_path.poses.size(); ++i)
    {
        const auto &p1 = blended_path.poses[i].pose.position;
        const auto &p2 = blended_path.poses[i + 1].pose.position;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, std::atan2(p2.y - p1.y, p2.x - p1.x));
        blended_path.poses[i].pose.orientation = tf2::toMsg(q);
    }
    if (blended_path.poses.size() >= 2)
    {
        blended_path.poses.back().pose.orientation =
            blended_path.poses[blended_path.poses.size() - 2].pose.orientation;
    }

    return blended_path;
}

double PIDController::compute_reacquire_measured_weight() const
{
    if (!has_corridor_confidence_)
    {
        return 0.0;
    }

    const double denominator = headland_reacquire_confidence_ - headland_reacquire_track_confidence_;
    if (denominator <= 1e-6)
    {
        return corridor_confidence_ >= headland_reacquire_confidence_ ? 1.0 : 0.0;
    }

    return std::clamp(
        (corridor_confidence_ - headland_reacquire_track_confidence_) / denominator,
        0.0,
        1.0);
}

double PIDController::reacquire_distance() const
{
    return std::hypot(current_x_ - reacquire_start_x_, current_y_ - reacquire_start_y_);
}

void PIDController::publish_headland_turn_path()
{
    if (!headland_turn_path_.poses.empty())
    {
        headland_turn_path_.header.stamp = this->now();
        headland_turn_path_pub_->publish(headland_turn_path_);
    }
}

void PIDController::publish_reacquire_reference_path(const nav_msgs::msg::Path &path)
{
    if (path.poses.empty())
    {
        return;
    }

    nav_msgs::msg::Path output_path = path;
    output_path.header.stamp = this->now();
    reacquire_reference_path_pub_->publish(output_path);
}

void PIDController::publish_navigation_mode()
{
    std_msgs::msg::String msg;
    msg.data = navigation_mode_name();
    navigation_mode_pub_->publish(msg);
}

void PIDController::publish_control_state(const std::string &state)
{
    const bool changed = state != last_control_state_;
    std_msgs::msg::String msg;
    msg.data = state;
    control_state_pub_->publish(msg);
    if (changed)
    {
        last_control_state_ = state;
        RCLCPP_INFO(this->get_logger(), "控制安全状态: %s", state.c_str());
    }
}

void PIDController::publish_quality_factor(double factor)
{
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(std::clamp(factor, 0.0, 1.0));
    quality_factor_pub_->publish(msg);
}

std::string PIDController::navigation_mode_name() const
{
    switch (navigation_mode_)
    {
    case NavigationMode::ROW_FOLLOW:
        return "ROW_FOLLOW";
    case NavigationMode::U_TURN:
        return "U_TURN";
    case NavigationMode::NEXT_ROW_REACQUIRE:
        return "NEXT_ROW_REACQUIRE";
    }
    return "UNKNOWN";
}

void PIDController::publish_target_marker(const geometry_msgs::msg::PointStamped &point)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom"; // 统一使用 odom 坐标系以确保一致性
    marker.header.stamp = this->now();
    marker.ns = "target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point.point;
    marker.pose.position.z = 0.2; // 稍微抬高一点，便于在地面上看到
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    target_marker_pub_->publish(marker);
}

geometry_msgs::msg::Twist PIDController::calculate_control_command()
{
    geometry_msgs::msg::Twist cmd_vel;

    // 调试模式：输出固定速度测试转向
    if (debug_mode_)
    {
        cmd_vel.linear.x = 0.1;
        cmd_vel.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "调试模式：线速度=0.1m/s, 角速度=0");
        return cmd_vel;
    }

    // 1. 寻找目标点（中心线前方target_distance_米处的点）
    target_point_ = find_target_point();
    publish_target_marker(target_point_); // 可视化目标点

    // 2. 计算时间差
    rclcpp::Time current_time = this->now();
    double dt = 0.0;

    // rclcpp::Time 的默认构造使用 ROS_TIME，而 this->now() 在当前配置下为
    // SYSTEM_TIME。首次控制周期或时钟源切换时不做相减，避免异常退出。
    if (!first_run_ && current_time.get_clock_type() == last_time_.get_clock_type())
    {
        dt = (current_time - last_time_).seconds();
    }
    else
    {
        dt = 0.05; // 假设50ms
        first_run_ = false;
    }
    last_time_ = current_time;

    // 3. 横向误差使用车辆到最近路径线段投影点的有符号垂距。
    //    不能使用车辆到前视点切线的距离，否则曲线路径会产生反向假误差。
    const double lateral_error = compute_lateral_error_to_path();

    // 4. 航向误差来自前视目标点处的中心线切线。
    const double heading_error =
        normalize_angle(target_path_heading_ - current_yaw_);

    std_msgs::msg::Float32 lateral_error_msg;
    lateral_error_msg.data = static_cast<float>(lateral_error);
    lateral_error_pub_->publish(lateral_error_msg);
    std_msgs::msg::Float32 heading_error_msg;
    heading_error_msg.data = static_cast<float>(heading_error);
    heading_error_pub_->publish(heading_error_msg);

    // 6. 使用PID控制器计算控制输出
    // 横向控制（主要影响转向）
    double lateral_control = compute_pid(lateral_error, dt, lateral_integral_, lateral_previous_error_,
                                         lateral_kp_, lateral_ki_, lateral_kd_);

    // 航向控制（辅助调整转向）
    double heading_control = compute_pid(heading_error, dt, heading_integral_, heading_previous_error_,
                                         heading_kp_, heading_ki_, heading_kd_);

    const double confidence_factor = compute_confidence_factor();
    const double safety_factor = compute_safety_factor();
    const double quality_factor = std::min(confidence_factor, safety_factor);
    publish_quality_factor(quality_factor);
    if (safety_factor <= 0.0)
    {
        return cmd_vel;
    }

    // 7. 组合控制输出
    double angular_vel = lateral_control + heading_control;
    const double current_max_angular_speed =
        std::max(0.1, max_angular_speed_ * quality_factor);
    angular_vel = limit_angular_command(
        angular_vel, current_max_angular_speed, dt);

    // 8. 计算线速度：中心线质量、安全裕度和转向幅度共同调速
    const double turning_factor = use_quality_aware_control_
                                      ? std::clamp(
                                            1.0 - 0.5 * std::abs(angular_vel) /
                                                      std::max(current_max_angular_speed, 1e-3),
                                            0.2,
                                            1.0)
                                      : 1.0;
    double linear_speed = max_linear_speed_ * confidence_factor * safety_factor * turning_factor;
    double mode_speed_limit = max_linear_speed_;
    if (navigation_mode_ == NavigationMode::U_TURN)
    {
        mode_speed_limit = headland_turn_linear_speed_;
    }
    else if (navigation_mode_ == NavigationMode::NEXT_ROW_REACQUIRE)
    {
        mode_speed_limit = headland_reacquire_search_speed_;
    }
    linear_speed = std::min(linear_speed, mode_speed_limit);
    if (linear_speed > 1e-6)
    {
        const double quality_min_speed = min_linear_speed_ * std::min(confidence_factor, safety_factor);
        linear_speed = std::max(linear_speed, quality_min_speed);
        linear_speed = std::min(linear_speed, mode_speed_limit);
    }

    // 9. 赋值控制指令
    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = angular_vel;

    // 调试日志
    RCLCPP_DEBUG(this->get_logger(),
                 "目标距离: %.2f, 横向误差: %.2f, 航向误差: %.2f, 线速度: %.2f, 角速度: %.2f, 置信因子: %.2f, 安全因子: %.2f",
                 target_distance_, lateral_error, heading_error, linear_speed, angular_vel,
                 confidence_factor, safety_factor);

    return cmd_vel;
}
