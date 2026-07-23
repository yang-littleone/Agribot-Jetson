#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "turn_on_32chassis/wheel_trust_monitor.hpp"

namespace
{

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return
    std::isfinite(pose.position.x) &&
    std::isfinite(pose.position.y) &&
    std::isfinite(pose.position.z) &&
    std::isfinite(pose.orientation.x) &&
    std::isfinite(pose.orientation.y) &&
    std::isfinite(pose.orientation.z) &&
    std::isfinite(pose.orientation.w);
}

bool finite_twist(const geometry_msgs::msg::Twist & twist)
{
  return
    std::isfinite(twist.linear.x) &&
    std::isfinite(twist.linear.y) &&
    std::isfinite(twist.linear.z) &&
    std::isfinite(twist.angular.x) &&
    std::isfinite(twist.angular.y) &&
    std::isfinite(twist.angular.z);
}

tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion rotation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z,
    pose.orientation.w);
  rotation.normalize();
  return tf2::Transform(
    rotation,
    tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::msg::Pose transform_to_planar_pose(const tf2::Transform & transform)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = 0.0;
  pose.orientation = tf2::toMsg(rotation);
  return pose;
}

double transform_yaw(const tf2::Transform & transform)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  return yaw;
}

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
}

}  // namespace

class SlipAwareOdometry : public rclcpp::Node
{
public:
  SlipAwareOdometry()
  : Node("slip_aware_odometry")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    wheel_topic_ = declare_parameter<std::string>("wheel_topic", "/wheel/odom");
    lio_input_topic_ = declare_parameter<std::string>("lio_input_topic", "/Odometry");
    lio_output_topic_ =
      declare_parameter<std::string>("lio_output_topic", "/lio/odom");
    validated_wheel_topic_ =
      declare_parameter<std::string>(
      "validated_wheel_topic", "/wheel/odom_validated");
    fused_input_topic_ =
      declare_parameter<std::string>(
      "fused_input_topic", "/odometry/fused_internal");
    final_odom_topic_ =
      declare_parameter<std::string>("final_odom_topic", "/odometry/filtered");

    comparison_window_ = declare_parameter<double>("comparison_window", 0.5);
    maximum_sync_delta_ = declare_parameter<double>("maximum_sync_delta", 0.15);
    wheel_lio_max_age_ = declare_parameter<double>("wheel_lio_max_age", 0.20);
    lio_timeout_ = declare_parameter<double>("lio_timeout", 0.50);
    output_frequency_ = declare_parameter<double>("output_frequency", 50.0);
    maximum_prediction_horizon_ =
      declare_parameter<double>("maximum_prediction_horizon", 0.15);
    maximum_wheel_prediction_distance_ =
      declare_parameter<double>("maximum_wheel_prediction_distance", 0.06);
    maximum_lio_latency_ =
      declare_parameter<double>("maximum_lio_latency", 0.30);
    stationary_hold_time_ =
      declare_parameter<double>("stationary_hold_time", 0.30);
    stationary_lio_speed_threshold_ =
      declare_parameter<double>("stationary_lio_speed_threshold", 0.025);
    stationary_lio_release_speed_ =
      declare_parameter<double>("stationary_lio_release_speed", 0.05);
    stationary_wheel_linear_threshold_ =
      declare_parameter<double>("stationary_wheel_linear_threshold", 0.01);
    stationary_wheel_angular_threshold_ =
      declare_parameter<double>("stationary_wheel_angular_threshold", 0.02);
    stationary_release_distance_ =
      declare_parameter<double>("stationary_release_distance", 0.02);
    instant_linear_residual_threshold_ =
      declare_parameter<double>("instant_linear_residual_threshold", 0.20);
    instant_linear_recovery_threshold_ =
      declare_parameter<double>("instant_linear_recovery_threshold", 0.10);
    instant_yaw_rate_residual_threshold_ =
      declare_parameter<double>("instant_yaw_rate_residual_threshold", 0.50);
    instant_yaw_rate_recovery_threshold_ =
      declare_parameter<double>("instant_yaw_rate_recovery_threshold", 0.25);
    minimum_observable_speed_ =
      declare_parameter<double>("minimum_observable_speed", 0.05);
    instant_mismatch_variance_scale_ =
      declare_parameter<double>("instant_mismatch_variance_scale", 100.0);
    lio_max_position_variance_ =
      declare_parameter<double>("lio_max_position_variance", 0.25);
    lio_pose_variance_floor_ =
      declare_parameter<double>("lio_pose_variance_floor", 0.001);
    lio_twist_variance_ =
      declare_parameter<double>("lio_twist_variance", 0.0025);
    normal_wheel_linear_variance_ =
      declare_parameter<double>("normal_wheel_linear_variance", 0.01);
    normal_wheel_angular_variance_ =
      declare_parameter<double>("normal_wheel_angular_variance", 0.02);
    frozen_variance_ = declare_parameter<double>("frozen_variance", 1000.0);
    twist_filter_alpha_ = declare_parameter<double>("twist_filter_alpha", 0.5);

    turn_on_32chassis::WheelTrustConfig monitor_config;
    monitor_config.translation_threshold =
      declare_parameter<double>("translation_residual_threshold", 0.03);
    monitor_config.yaw_threshold =
      declare_parameter<double>("yaw_residual_threshold", 0.12);
    monitor_config.recovery_translation_threshold =
      declare_parameter<double>("recovery_translation_threshold", 0.015);
    monitor_config.recovery_yaw_threshold =
      declare_parameter<double>("recovery_yaw_threshold", 0.06);
    monitor_config.minimum_motion =
      declare_parameter<double>("minimum_observable_motion", 0.015);
    monitor_config.confirmation_time =
      declare_parameter<double>("slip_confirmation_time", 0.5);
    monitor_config.recovery_time =
      declare_parameter<double>("wheel_recovery_time", 1.0);
    monitor_config.recovery_ramp_time =
      declare_parameter<double>("wheel_recovery_ramp_time", 1.0);
    monitor_config.suspect_variance_scale =
      declare_parameter<double>("suspect_variance_scale", 25.0);
    monitor_config.rejected_variance_scale =
      declare_parameter<double>("rejected_variance_scale", 1.0e6);
    wheel_monitor_ =
      turn_on_32chassis::WheelTrustMonitor(monitor_config);
    // Before the first LIO frame there is no safe reference for wheel motion.
    wheel_monitor_.start_rejected(now().seconds());

    const double base_to_lio_body_x =
      declare_parameter<double>("base_to_lio_body.x", 0.04263);
    const double base_to_lio_body_y =
      declare_parameter<double>("base_to_lio_body.y", 0.02338);
    const double base_to_lio_body_z =
      declare_parameter<double>("base_to_lio_body.z", 0.07088);
    const double base_to_lio_body_roll =
      declare_parameter<double>("base_to_lio_body.roll", 0.0);
    const double base_to_lio_body_pitch =
      declare_parameter<double>("base_to_lio_body.pitch", 0.0);
    const double base_to_lio_body_yaw =
      declare_parameter<double>("base_to_lio_body.yaw", 0.0);
    tf2::Quaternion base_to_lio_rotation;
    base_to_lio_rotation.setRPY(
      base_to_lio_body_roll, base_to_lio_body_pitch,
      base_to_lio_body_yaw);
    base_to_lio_body_ = tf2::Transform(
      base_to_lio_rotation,
      tf2::Vector3(
        base_to_lio_body_x, base_to_lio_body_y,
        base_to_lio_body_z));

    validated_wheel_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(validated_wheel_topic_, 20);
    lio_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(lio_output_topic_, 20);
    final_odom_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>(final_odom_topic_, 20);

    wheel_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic_, 20,
      std::bind(&SlipAwareOdometry::wheel_callback, this, std::placeholders::_1));
    lio_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      lio_input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlipAwareOdometry::lio_callback, this, std::placeholders::_1));
    fused_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      fused_input_topic_, 20,
      std::bind(&SlipAwareOdometry::fused_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto output_period = std::chrono::duration<double>(
      1.0 / std::max(output_frequency_, 1.0));
    output_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(output_period),
      std::bind(&SlipAwareOdometry::publish_final_output, this));

    RCLCPP_INFO(
      get_logger(),
      "Slip-aware odometry: wheel=%s lio=%s fused=%s final=%s",
      wheel_topic_.c_str(), lio_input_topic_.c_str(),
      fused_input_topic_.c_str(), final_odom_topic_.c_str());
  }

private:
  struct MotionPair
  {
    double stamp{0.0};
    tf2::Transform wheel;
    tf2::Transform lio;
  };

  struct TimedTransform
  {
    double stamp{0.0};
    tf2::Transform transform;
  };

  bool lio_recent(double maximum_age) const
  {
    return have_lio_ &&
           (now() - last_lio_receive_time_).seconds() <= maximum_age;
  }

  bool lio_covariance_is_usable(const nav_msgs::msg::Odometry & message) const
  {
    const double x_variance = message.pose.covariance[0];
    const double y_variance = message.pose.covariance[7];
    return
      std::isfinite(x_variance) && std::isfinite(y_variance) &&
      x_variance >= 0.0 && y_variance >= 0.0 &&
      x_variance <= lio_max_position_variance_ &&
      y_variance <= lio_max_position_variance_;
  }

  void wheel_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!finite_pose(message->pose.pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring wheel odometry with a non-finite pose");
      return;
    }

    latest_wheel_ = *message;
    latest_wheel_transform_ = pose_to_transform(message->pose.pose);
    have_wheel_ = true;
    const double wheel_stamp = stamp_seconds(message->header.stamp);
    if (
      wheel_history_.empty() ||
      wheel_stamp > wheel_history_.back().stamp)
    {
      wheel_history_.push_back(
        TimedTransform{wheel_stamp, latest_wheel_transform_});
      while (
        wheel_history_.size() > 2 &&
        wheel_stamp - wheel_history_.front().stamp > 2.0)
      {
        wheel_history_.pop_front();
      }
    }

    // Do not wait for a full displacement window when the latest observed
    // LIO velocity already contradicts the new wheel speed. This is
    // particularly important for a stationary robot whose wheels suddenly
    // start spinning against an obstacle.
    if (have_lio_ && lio_recent(wheel_lio_max_age_)) {
      update_instant_wheel_consistency(latest_lio_odom_);
    }

    // Never fall back to wheel odometry when the independent LIO reference
    // is absent or stale.
    if (!lio_recent(wheel_lio_max_age_) || wheel_monitor_.reject_wheel()) {
      return;
    }

    nav_msgs::msg::Odometry validated = *message;
    validated.header.frame_id = odom_frame_;
    validated.child_frame_id = base_frame_;
    validated.pose.covariance.fill(0.0);
    validated.twist.covariance.fill(0.0);

    for (std::size_t index = 0; index < 6; ++index) {
      validated.pose.covariance[index * 6 + index] = 1.0e6;
      validated.twist.covariance[index * 6 + index] = 1.0e6;
    }

    double scale = wheel_monitor_.variance_scale(now().seconds());
    if (instant_wheel_mismatch_) {
      scale = std::max(scale, instant_mismatch_variance_scale_);
    }
    validated.twist.covariance[0] =
      std::min(1.0e6, normal_wheel_linear_variance_ * scale);
    validated.twist.covariance[7] =
      std::min(1.0e6, normal_wheel_linear_variance_ * scale);
    validated.twist.covariance[35] =
      std::min(1.0e6, normal_wheel_angular_variance_ * scale);
    validated_wheel_publisher_->publish(validated);
    publish_final_output();
  }

  bool wheel_pose_at(
    double requested_stamp, tf2::Transform & transform) const
  {
    double best_delta = std::numeric_limits<double>::infinity();
    bool found = false;
    for (auto iterator = wheel_history_.rbegin();
      iterator != wheel_history_.rend(); ++iterator)
    {
      const double delta = std::abs(iterator->stamp - requested_stamp);
      if (delta < best_delta) {
        best_delta = delta;
        transform = iterator->transform;
        found = true;
      }
      if (
        iterator->stamp < requested_stamp &&
        delta > best_delta)
      {
        break;
      }
    }
    return found && best_delta <= maximum_sync_delta_;
  }

  void lio_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (
      !finite_pose(message->pose.pose) ||
      !finite_twist(message->twist.twist) ||
      !lio_covariance_is_usable(*message))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid or degraded FAST-LIO odometry");
      return;
    }

    const double message_stamp = stamp_seconds(message->header.stamp);
    if (have_lio_stamp_ && message_stamp <= last_lio_stamp_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring out-of-order FAST-LIO odometry");
      return;
    }

    const double raw_latency = now().seconds() - message_stamp;
    if (!have_lio_latency_baseline_) {
      minimum_lio_latency_ = raw_latency;
      have_lio_latency_baseline_ = true;
    } else {
      minimum_lio_latency_ = std::min(minimum_lio_latency_, raw_latency);
    }
    // When sensor and ROS clocks share an epoch, raw_latency is the actual
    // end-to-end delay. Otherwise the minimum observed offset is removed so
    // that a growing processing backlog is still detected.
    const double effective_latency =
      std::abs(raw_latency) < 60.0 ?
      raw_latency :
      raw_latency - minimum_lio_latency_;
    if (
      maximum_lio_latency_ > 0.0 &&
      effective_latency > maximum_lio_latency_)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Dropping stale FAST-LIO odometry: latency %.3f s exceeds %.3f s",
        effective_latency, maximum_lio_latency_);
      return;
    }

    const tf2::Transform lio_to_lio_body =
      pose_to_transform(message->pose.pose);
    const tf2::Transform lio_to_base =
      lio_to_lio_body * base_to_lio_body_.inverse();

    if (!lio_alignment_initialized_) {
      const tf2::Transform target =
        have_last_final_ ?
        pose_to_transform(last_final_.pose.pose) :
        tf2::Transform::getIdentity();
      odom_from_lio_ = target * lio_to_base.inverse();
      lio_alignment_initialized_ = true;
      motion_history_.clear();
      have_previous_lio_pose_ = false;
      filtered_lio_vx_ = 0.0;
      filtered_lio_vy_ = 0.0;
      filtered_lio_yaw_rate_ = 0.0;
      stationary_hold_active_ = false;
      stationary_candidate_ = false;
      instant_wheel_mismatch_ = false;
      if (have_last_final_) {
        wheel_monitor_.start_rejected(now().seconds());
      } else {
        // Direct LIO position limits any startup wheel error to one scan
        // interval, so allow high-rate wheel interpolation immediately.
        wheel_monitor_.reset();
      }
      RCLCPP_INFO(get_logger(), "Aligned FAST-LIO local frame to odom");
    }

    const tf2::Transform odom_to_base = odom_from_lio_ * lio_to_base;
    nav_msgs::msg::Odometry standardized;
    standardized.header = message->header;
    standardized.header.frame_id = odom_frame_;
    standardized.child_frame_id = base_frame_;
    standardized.pose.pose = transform_to_planar_pose(odom_to_base);
    standardized.pose.covariance.fill(0.0);
    standardized.twist.covariance.fill(0.0);
    for (std::size_t index = 0; index < 6; ++index) {
      standardized.pose.covariance[index * 6 + index] = 1.0e6;
      standardized.twist.covariance[index * 6 + index] = 1.0e6;
    }
    standardized.pose.covariance[0] =
      std::max(lio_pose_variance_floor_, message->pose.covariance[0]);
    standardized.pose.covariance[7] =
      std::max(lio_pose_variance_floor_, message->pose.covariance[7]);
    standardized.pose.covariance[35] =
      std::max(0.01, message->pose.covariance[35]);

    const bool lio_velocity_is_observable = have_previous_lio_pose_;
    if (have_previous_lio_pose_) {
      const double delta_time = message_stamp - last_lio_stamp_;
      if (delta_time > 1.0e-4) {
        const tf2::Transform relative =
          previous_lio_pose_.inverse() * odom_to_base;
        // FAST-LIO publishes its current ESKF velocity in the child/body
        // frame. Prefer it over pose differencing because it is available at
        // the current scan and does not add another scan of phase delay.
        const double measured_vx = message->twist.twist.linear.x;
        const double measured_vy = message->twist.twist.linear.y;
        const double measured_yaw_rate =
          std::abs(message->twist.twist.angular.z) > 1.0e-6 ?
          message->twist.twist.angular.z :
          normalize_angle(transform_yaw(relative)) / delta_time;
        filtered_lio_vx_ =
          twist_filter_alpha_ * measured_vx +
          (1.0 - twist_filter_alpha_) * filtered_lio_vx_;
        filtered_lio_vy_ =
          twist_filter_alpha_ * measured_vy +
          (1.0 - twist_filter_alpha_) * filtered_lio_vy_;
        filtered_lio_yaw_rate_ =
          twist_filter_alpha_ * measured_yaw_rate +
          (1.0 - twist_filter_alpha_) * filtered_lio_yaw_rate_;
      }
    }
    standardized.twist.twist.linear.x = filtered_lio_vx_;
    standardized.twist.twist.linear.y = filtered_lio_vy_;
    standardized.twist.twist.angular.z = filtered_lio_yaw_rate_;
    standardized.twist.covariance[0] = lio_twist_variance_;
    standardized.twist.covariance[7] = lio_twist_variance_;
    standardized.twist.covariance[35] = 0.02;

    previous_lio_pose_ = odom_to_base;
    have_previous_lio_pose_ = true;
    last_lio_stamp_ = message_stamp;
    have_lio_stamp_ = true;
    last_lio_receive_time_ = now();
    latest_lio_transform_ = odom_to_base;
    latest_lio_odom_ = standardized;
    have_lio_ = true;
    last_lio_message_time_ = rclcpp::Time(message->header.stamp);

    tf2::Transform synchronized_wheel;
    if (wheel_pose_at(message_stamp, synchronized_wheel)) {
      wheel_anchor_at_lio_ = synchronized_wheel;
      have_wheel_anchor_at_lio_ = true;
    } else {
      have_wheel_anchor_at_lio_ = false;
    }

    if (lio_velocity_is_observable) {
      update_instant_wheel_consistency(standardized);
    }
    update_stationary_hold(standardized);
    lio_publisher_->publish(standardized);

    update_wheel_consistency(
      message_stamp, odom_to_base, synchronized_wheel,
      have_wheel_anchor_at_lio_);
    publish_final_output();
  }

  void update_instant_wheel_consistency(
    const nav_msgs::msg::Odometry & lio_odometry)
  {
    if (!have_wheel_) {
      return;
    }

    const double wheel_speed = std::hypot(
      latest_wheel_.twist.twist.linear.x,
      latest_wheel_.twist.twist.linear.y);
    const double lio_speed = std::hypot(
      lio_odometry.twist.twist.linear.x,
      lio_odometry.twist.twist.linear.y);
    const double linear_residual = std::hypot(
      latest_wheel_.twist.twist.linear.x -
      lio_odometry.twist.twist.linear.x,
      latest_wheel_.twist.twist.linear.y -
      lio_odometry.twist.twist.linear.y);
    const double yaw_rate_residual = std::abs(
      latest_wheel_.twist.twist.angular.z -
      lio_odometry.twist.twist.angular.z);
    const bool observable =
      std::max(wheel_speed, lio_speed) >= minimum_observable_speed_;

    if (!observable) {
      instant_wheel_mismatch_ = false;
      return;
    }
    if (!instant_wheel_mismatch_) {
      instant_wheel_mismatch_ =
        linear_residual > instant_linear_residual_threshold_ ||
        yaw_rate_residual > instant_yaw_rate_residual_threshold_;
    } else {
      instant_wheel_mismatch_ =
        linear_residual > instant_linear_recovery_threshold_ ||
        yaw_rate_residual > instant_yaw_rate_recovery_threshold_;
    }
  }

  void update_stationary_hold(
    const nav_msgs::msg::Odometry & lio_odometry)
  {
    const double current_time = now().seconds();
    const double lio_speed = std::hypot(
      lio_odometry.twist.twist.linear.x,
      lio_odometry.twist.twist.linear.y);
    const double wheel_speed = have_wheel_ ?
      std::hypot(
      latest_wheel_.twist.twist.linear.x,
      latest_wheel_.twist.twist.linear.y) :
      0.0;
    const double wheel_yaw_rate = have_wheel_ ?
      std::abs(latest_wheel_.twist.twist.angular.z) :
      0.0;
    const bool wheel_is_stationary =
      !have_wheel_ ||
      (wheel_speed <= stationary_wheel_linear_threshold_ &&
      wheel_yaw_rate <= stationary_wheel_angular_threshold_);
    const bool trusted_wheel_motion =
      have_wheel_ && !instant_wheel_mismatch_ && !wheel_is_stationary;
    const double hold_displacement = std::hypot(
      lio_odometry.pose.pose.position.x - stationary_hold_x_,
      lio_odometry.pose.pose.position.y - stationary_hold_y_);

    if (stationary_hold_active_) {
      if (
        trusted_wheel_motion ||
        lio_speed > stationary_lio_release_speed_ ||
        hold_displacement > stationary_release_distance_)
      {
        stationary_hold_active_ = false;
        stationary_candidate_ = false;
        RCLCPP_INFO(
          get_logger(),
          "Released stationary odometry hold (LIO speed %.3f m/s, displacement %.3f m)",
          lio_speed, hold_displacement);
      }
      return;
    }

    const bool stationary_evidence =
      lio_speed <= stationary_lio_speed_threshold_ &&
      (wheel_is_stationary || instant_wheel_mismatch_);
    if (!stationary_evidence) {
      stationary_candidate_ = false;
      stationary_sample_count_ = 0;
      return;
    }

    if (!stationary_candidate_) {
      stationary_candidate_ = true;
      stationary_candidate_start_time_ = current_time;
      stationary_sum_x_ = lio_odometry.pose.pose.position.x;
      stationary_sum_y_ = lio_odometry.pose.pose.position.y;
      stationary_sample_count_ = 1;
    } else {
      stationary_sum_x_ += lio_odometry.pose.pose.position.x;
      stationary_sum_y_ += lio_odometry.pose.pose.position.y;
      ++stationary_sample_count_;
    }

    if (
      current_time - stationary_candidate_start_time_ >=
      stationary_hold_time_)
    {
      stationary_hold_x_ =
        stationary_sum_x_ / static_cast<double>(stationary_sample_count_);
      stationary_hold_y_ =
        stationary_sum_y_ / static_cast<double>(stationary_sample_count_);
      stationary_hold_active_ = true;
      stationary_candidate_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Holding stationary odometry at x=%.3f y=%.3f",
        stationary_hold_x_, stationary_hold_y_);
    }
  }

  void update_wheel_consistency(
    double lio_stamp, const tf2::Transform & lio_transform,
    const tf2::Transform & synchronized_wheel,
    bool have_synchronized_wheel)
  {
    if (!have_synchronized_wheel) {
      return;
    }

    motion_history_.push_back(
      MotionPair{lio_stamp, synchronized_wheel, lio_transform});
    // Keep the newest sample that still gives a full comparison window as
    // the anchor. Removing merely because the oldest sample crossed the
    // window can leave only a single scan interval and prevent comparison
    // forever when timestamps have small scheduling jitter.
    while (
      motion_history_.size() > 2 &&
      lio_stamp - motion_history_[1].stamp >= comparison_window_)
    {
      motion_history_.pop_front();
    }
    if (
      motion_history_.size() < 2 ||
      motion_history_.back().stamp - motion_history_.front().stamp <
      comparison_window_ * 0.7)
    {
      return;
    }

    const tf2::Transform wheel_delta =
      motion_history_.front().wheel.inverse() * motion_history_.back().wheel;
    const tf2::Transform lio_delta =
      motion_history_.front().lio.inverse() * motion_history_.back().lio;
    const double translation_error = std::hypot(
      wheel_delta.getOrigin().x() - lio_delta.getOrigin().x(),
      wheel_delta.getOrigin().y() - lio_delta.getOrigin().y());
    const double yaw_error = std::abs(normalize_angle(
        transform_yaw(wheel_delta) - transform_yaw(lio_delta)));
    const double wheel_motion = std::hypot(
      wheel_delta.getOrigin().x(), wheel_delta.getOrigin().y());
    const double lio_motion = std::hypot(
      lio_delta.getOrigin().x(), lio_delta.getOrigin().y());

    const auto old_state = wheel_monitor_.state();
    wheel_monitor_.update(
      now().seconds(), translation_error, yaw_error,
      std::max(wheel_motion, lio_motion));
    if (old_state != wheel_monitor_.state()) {
      RCLCPP_WARN(
        get_logger(),
        "Wheel trust state changed %d -> %d (translation residual %.3f m, yaw residual %.3f rad)",
        static_cast<int>(old_state), static_cast<int>(wheel_monitor_.state()),
        translation_error, yaw_error);
    }
  }

  void fused_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!finite_pose(message->pose.pose)) {
      return;
    }
    latest_fused_ = *message;
    have_fused_ = true;
    publish_final_output();
  }

  double lio_prediction_age(const rclcpp::Time & current_time) const
  {
    double message_age = (current_time - last_lio_message_time_).seconds();
    if (
      !std::isfinite(message_age) || message_age < 0.0 ||
      message_age > 2.0)
    {
      message_age = (current_time - last_lio_receive_time_).seconds();
    }
    return std::clamp(message_age, 0.0, maximum_prediction_horizon_);
  }

  tf2::Transform low_latency_lio_pose(const rclcpp::Time & current_time) const
  {
    tf2::Transform predicted = latest_lio_transform_;
    const double heading_yaw =
      transform_yaw(pose_to_transform(latest_fused_.pose.pose));
    tf2::Quaternion heading;
    heading.setRPY(0.0, 0.0, heading_yaw);

    double prediction_weight =
      wheel_monitor_.prediction_weight(current_time.seconds());
    if (instant_wheel_mismatch_) {
      prediction_weight = 0.0;
    }

    bool used_wheel_prediction = false;
    if (
      prediction_weight > 0.0 && have_wheel_anchor_at_lio_ &&
      have_wheel_)
    {
      const double wheel_prediction_time =
        stamp_seconds(latest_wheel_.header.stamp) - last_lio_stamp_;
      if (wheel_prediction_time >= 0.0) {
        used_wheel_prediction = true;
      }
      if (wheel_prediction_time > 1.0e-4) {
        const tf2::Transform wheel_delta =
          wheel_anchor_at_lio_.inverse() * latest_wheel_transform_;
        const double horizon_scale = std::min(
          1.0, maximum_prediction_horizon_ / wheel_prediction_time);
        const tf2::Vector3 local_delta(
          wheel_delta.getOrigin().x() * prediction_weight * horizon_scale,
          wheel_delta.getOrigin().y() * prediction_weight * horizon_scale,
          0.0);
        tf2::Vector3 bounded_delta = local_delta;
        const double prediction_distance = bounded_delta.length();
        if (
          maximum_wheel_prediction_distance_ > 0.0 &&
          prediction_distance > maximum_wheel_prediction_distance_)
        {
          bounded_delta *=
            maximum_wheel_prediction_distance_ / prediction_distance;
        }
        predicted.setOrigin(
          predicted.getOrigin() +
          tf2::Matrix3x3(heading) * bounded_delta);
      }
    }

    if (!used_wheel_prediction) {
      const double prediction_time = lio_prediction_age(current_time);
      const tf2::Vector3 local_delta(
        latest_lio_odom_.twist.twist.linear.x * prediction_time,
        latest_lio_odom_.twist.twist.linear.y * prediction_time,
        0.0);
      predicted.setOrigin(
        predicted.getOrigin() +
        tf2::Matrix3x3(heading) * local_delta);
    }
    return predicted;
  }

  void publish_final_output()
  {
    if (!have_lio_ || !have_fused_) {
      return;
    }

    const rclcpp::Time current_time = now();
    const double lio_age = (current_time - last_lio_receive_time_).seconds();
    nav_msgs::msg::Odometry output;

    if (lio_age > lio_timeout_) {
      if (!frozen_) {
        if (!have_last_final_) {
          return;
        }
        frozen_ = true;
        frozen_pose_ = pose_to_transform(last_final_.pose.pose);
        lio_alignment_initialized_ = false;
        have_previous_lio_pose_ = false;
        filtered_lio_vx_ = 0.0;
        filtered_lio_vy_ = 0.0;
        filtered_lio_yaw_rate_ = 0.0;
        have_lio_stamp_ = false;
        have_wheel_anchor_at_lio_ = false;
        instant_wheel_mismatch_ = true;
        motion_history_.clear();
        wheel_monitor_.start_rejected(now().seconds());
        RCLCPP_ERROR(
          get_logger(),
          "FAST-LIO timed out after %.3f s; freezing final odometry",
          lio_age);
      }
      output = last_final_;
      output.pose.pose = transform_to_planar_pose(frozen_pose_);
      output.twist.twist = geometry_msgs::msg::Twist();
      output.pose.covariance[0] = frozen_variance_;
      output.pose.covariance[7] = frozen_variance_;
      output.pose.covariance[35] = frozen_variance_;
      output.twist.covariance[0] = frozen_variance_;
      output.twist.covariance[7] = frozen_variance_;
      output.twist.covariance[35] = frozen_variance_;
    } else {
      output = latest_fused_;
      output.header.frame_id = odom_frame_;
      output.child_frame_id = base_frame_;

      if (frozen_) {
        frozen_ = false;
        RCLCPP_INFO(
          get_logger(),
          "FAST-LIO recovered; continuing from the frozen odom pose");
      }

      tf2::Transform output_pose = low_latency_lio_pose(current_time);
      if (stationary_hold_active_) {
        output_pose.getOrigin().setX(stationary_hold_x_);
        output_pose.getOrigin().setY(stationary_hold_y_);
      }
      output.pose.pose = transform_to_planar_pose(output_pose);
      output.pose.pose.orientation =
        transform_to_planar_pose(
        pose_to_transform(latest_fused_.pose.pose)).orientation;
      const double prediction_variance =
        0.01 * lio_prediction_age(current_time);
      output.pose.covariance[0] =
        std::max(
        lio_pose_variance_floor_,
        latest_lio_odom_.pose.covariance[0]) + prediction_variance;
      output.pose.covariance[7] =
        std::max(
        lio_pose_variance_floor_,
        latest_lio_odom_.pose.covariance[7]) + prediction_variance;

      const double wheel_prediction_weight =
        instant_wheel_mismatch_ ? 0.0 :
        wheel_monitor_.prediction_weight(current_time.seconds());
      if (stationary_hold_active_) {
        output.twist.twist.linear.x = 0.0;
        output.twist.twist.linear.y = 0.0;
        output.twist.covariance[0] =
          latest_lio_odom_.twist.covariance[0];
        output.twist.covariance[7] =
          latest_lio_odom_.twist.covariance[7];
      } else if (wheel_prediction_weight <= 0.0) {
        output.twist.twist.linear =
          latest_lio_odom_.twist.twist.linear;
        output.twist.covariance[0] =
          latest_lio_odom_.twist.covariance[0];
        output.twist.covariance[7] =
          latest_lio_odom_.twist.covariance[7];
      }
    }

    output.header.stamp = current_time;
    final_odom_publisher_->publish(output);
    broadcast_final_transform(output);
    last_final_ = output;
    have_last_final_ = true;
  }

  void broadcast_final_transform(const nav_msgs::msg::Odometry & odometry)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odometry.pose.pose.position.x;
    transform.transform.translation.y = odometry.pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  std::string odom_frame_;
  std::string base_frame_;
  std::string wheel_topic_;
  std::string lio_input_topic_;
  std::string lio_output_topic_;
  std::string validated_wheel_topic_;
  std::string fused_input_topic_;
  std::string final_odom_topic_;

  double comparison_window_{0.5};
  double maximum_sync_delta_{0.15};
  double wheel_lio_max_age_{0.20};
  double lio_timeout_{0.50};
  double output_frequency_{50.0};
  double maximum_prediction_horizon_{0.15};
  double maximum_wheel_prediction_distance_{0.06};
  double maximum_lio_latency_{0.30};
  double stationary_hold_time_{0.30};
  double stationary_lio_speed_threshold_{0.025};
  double stationary_lio_release_speed_{0.05};
  double stationary_wheel_linear_threshold_{0.01};
  double stationary_wheel_angular_threshold_{0.02};
  double stationary_release_distance_{0.02};
  double instant_linear_residual_threshold_{0.20};
  double instant_linear_recovery_threshold_{0.10};
  double instant_yaw_rate_residual_threshold_{0.50};
  double instant_yaw_rate_recovery_threshold_{0.25};
  double minimum_observable_speed_{0.05};
  double instant_mismatch_variance_scale_{100.0};
  double lio_max_position_variance_{0.25};
  double lio_pose_variance_floor_{0.001};
  double lio_twist_variance_{0.0025};
  double normal_wheel_linear_variance_{0.01};
  double normal_wheel_angular_variance_{0.02};
  double frozen_variance_{1000.0};
  double twist_filter_alpha_{0.5};

  turn_on_32chassis::WheelTrustMonitor wheel_monitor_;
  tf2::Transform base_to_lio_body_{tf2::Transform::getIdentity()};
  tf2::Transform odom_from_lio_{tf2::Transform::getIdentity()};
  tf2::Transform previous_lio_pose_{tf2::Transform::getIdentity()};
  tf2::Transform latest_lio_transform_{tf2::Transform::getIdentity()};
  tf2::Transform latest_wheel_transform_{tf2::Transform::getIdentity()};
  tf2::Transform wheel_anchor_at_lio_{tf2::Transform::getIdentity()};
  tf2::Transform frozen_pose_{tf2::Transform::getIdentity()};

  bool lio_alignment_initialized_{false};
  bool have_previous_lio_pose_{false};
  bool have_lio_{false};
  bool have_lio_stamp_{false};
  bool have_wheel_{false};
  bool have_wheel_anchor_at_lio_{false};
  bool have_fused_{false};
  bool have_last_final_{false};
  bool frozen_{false};
  bool instant_wheel_mismatch_{false};
  bool have_lio_latency_baseline_{false};
  bool stationary_candidate_{false};
  bool stationary_hold_active_{false};
  double last_lio_stamp_{0.0};
  double minimum_lio_latency_{0.0};
  double filtered_lio_vx_{0.0};
  double filtered_lio_vy_{0.0};
  double filtered_lio_yaw_rate_{0.0};
  double stationary_candidate_start_time_{0.0};
  double stationary_sum_x_{0.0};
  double stationary_sum_y_{0.0};
  double stationary_hold_x_{0.0};
  double stationary_hold_y_{0.0};
  std::size_t stationary_sample_count_{0};
  rclcpp::Time last_lio_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lio_message_time_{0, 0, RCL_ROS_TIME};

  nav_msgs::msg::Odometry latest_wheel_;
  nav_msgs::msg::Odometry latest_lio_odom_;
  nav_msgs::msg::Odometry latest_fused_;
  nav_msgs::msg::Odometry last_final_;
  std::deque<MotionPair> motion_history_;
  std::deque<TimedTransform> wheel_history_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr validated_wheel_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lio_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr final_odom_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fused_subscription_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlipAwareOdometry>());
  rclcpp::shutdown();
  return 0;
}
