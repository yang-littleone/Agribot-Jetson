#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuSourceSelector : public rclcpp::Node
{
public:
  ImuSourceSelector()
  : Node("imu_source_selector")
  {
    orientation_variance_ = declare_parameter<double>("orientation_variance", 0.0025);
    angular_velocity_variance_ =
      declare_parameter<double>("angular_velocity_variance", 0.0004);
    linear_acceleration_variance_ =
      declare_parameter<double>("linear_acceleration_variance", 0.04);

    publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/selected", rclcpp::QoS(10));
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/input", rclcpp::SensorDataQoS(),
      std::bind(&ImuSourceSelector::imu_callback, this, std::placeholders::_1));
  }

private:
  static bool covariance_is_unset(const std::array<double, 9> & covariance)
  {
    return std::all_of(
      covariance.begin(), covariance.end(),
      [](double value) {return value == 0.0;});
  }

  static void set_diagonal_covariance(std::array<double, 9> & covariance, double variance)
  {
    covariance.fill(0.0);
    covariance[0] = variance;
    covariance[4] = variance;
    covariance[8] = variance;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr input)
  {
    sensor_msgs::msg::Imu output = *input;

    const double norm = std::sqrt(
      output.orientation.x * output.orientation.x +
      output.orientation.y * output.orientation.y +
      output.orientation.z * output.orientation.z +
      output.orientation.w * output.orientation.w);

    if (!std::isfinite(norm) || norm < 1e-6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring IMU message with an invalid orientation quaternion");
      return;
    }

    output.orientation.x /= norm;
    output.orientation.y /= norm;
    output.orientation.z /= norm;
    output.orientation.w /= norm;

    if (covariance_is_unset(output.orientation_covariance)) {
      set_diagonal_covariance(output.orientation_covariance, orientation_variance_);
    }
    if (covariance_is_unset(output.angular_velocity_covariance)) {
      set_diagonal_covariance(
        output.angular_velocity_covariance, angular_velocity_variance_);
    }
    if (covariance_is_unset(output.linear_acceleration_covariance)) {
      set_diagonal_covariance(
        output.linear_acceleration_covariance, linear_acceleration_variance_);
    }

    publisher_->publish(output);
  }

  double orientation_variance_;
  double angular_velocity_variance_;
  double linear_acceleration_variance_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSourceSelector>());
  rclcpp::shutdown();
  return 0;
}
