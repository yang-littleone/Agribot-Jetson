#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

// 将一条局部测试轨迹固定到启动瞬间的 /odometry/filtered 位姿。
// 因而无需手工填写 odom 坐标；只要车头沿场地 3.6 m 的长边摆放即可。
class TestPathPublisher : public rclcpp::Node
{
public:
    TestPathPublisher() : Node("test_path_publisher")
    {
        profile_ = declare_parameter<std::string>("profile", "straight_half_circle");
        topic_ = declare_parameter<std::string>("path_topic", "/corn_row_center_line");
        sample_spacing_ = declare_parameter<double>("sample_spacing", 0.05);
        straight_length_ = declare_parameter<double>("straight_length", 3.0);
        lead_in_length_ = declare_parameter<double>("lead_in_length", 0.60);
        s_curve_length_ = declare_parameter<double>("s_curve_length", 1.80);
        s_amplitude_ = declare_parameter<double>("s_amplitude", 0.20);
        straight_before_arc_ = declare_parameter<double>("straight_before_arc", 1.80);
        arc_radius_ = declare_parameter<double>("arc_radius", 0.25);
        arc_angle_degrees_ = declare_parameter<double>("arc_angle_degrees", 180.0);
        arc_exit_length_ = declare_parameter<double>("arc_exit_length", 0.20);
        const auto arc_direction = declare_parameter<std::string>("arc_direction", "left");
        arc_turn_sign_ = (arc_direction == "right") ? -1.0 : 1.0;
        publish_period_s_ = declare_parameter<double>("publish_period", 0.10);

        sample_spacing_ = std::max(0.02, sample_spacing_);
        straight_length_ = std::max(0.30, straight_length_);
        lead_in_length_ = std::max(0.0, lead_in_length_);
        s_curve_length_ = std::max(0.20, s_curve_length_);
        s_amplitude_ = std::max(0.0, s_amplitude_);
        straight_before_arc_ = std::max(0.20, straight_before_arc_);
        arc_radius_ = std::max(0.05, arc_radius_);
        arc_angle_degrees_ = std::clamp(arc_angle_degrees_, 10.0, 180.0);
        arc_exit_length_ = std::max(0.0, arc_exit_length_);
        publish_period_s_ = std::max(0.02, publish_period_s_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        path_pub_ = create_publisher<nav_msgs::msg::Path>(topic_, qos);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&TestPathPublisher::odom_callback, this, std::placeholders::_1));
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(publish_period_s_)),
            std::bind(&TestPathPublisher::publish_path, this));

        RCLCPP_INFO(get_logger(),
                    "等待 /odometry/filtered 以固定测试路径：profile=%s, topic=%s",
                    profile_.c_str(), topic_.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (anchored_)
        {
            return;
        }

        origin_x_ = msg->pose.pose.position.x;
        origin_y_ = msg->pose.pose.position.y;
        frame_id_ = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll = 0.0;
        double pitch = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, origin_yaw_);

        build_path();
        anchored_ = true;
        RCLCPP_INFO(get_logger(),
                    "测试路径已固定在 odom=(%.2f, %.2f, %.1f deg)，共 %zu 点，终点距起点 %.2f m",
                    origin_x_, origin_y_, origin_yaw_ * 180.0 / kPi,
                    path_.poses.size(), path_length_);
    }

    void build_path()
    {
        path_.header.frame_id = frame_id_;
        path_.poses.clear();
        if (profile_ == "straight")
        {
            path_length_ = straight_length_;
        }
        else if (profile_ == "straight_arc" || profile_ == "straight_half_circle")
        {
            path_length_ = straight_before_arc_ +
                arc_radius_ * arc_angle_degrees_ * kPi / 180.0 + arc_exit_length_;
        }
        else
        {
            path_length_ = 2.0 * lead_in_length_ + s_curve_length_;
        }

        const std::size_t samples = static_cast<std::size_t>(std::ceil(path_length_ / sample_spacing_));
        for (std::size_t i = 0; i <= samples; ++i)
        {
            const double local_x = std::min(path_length_, static_cast<double>(i) * sample_spacing_);
            double local_y = 0.0;
            double path_x = local_x;
            double dy_dx = 0.0;
            double local_heading = 0.0;

            if (profile_ == "straight_arc" || profile_ == "straight_half_circle")
            {
                const double arc_angle = arc_angle_degrees_ * kPi / 180.0;
                const double arc_length = arc_radius_ * arc_angle;
                if (local_x <= straight_before_arc_)
                {
                    path_x = local_x;
                }
                else if (local_x <= straight_before_arc_ + arc_length)
                {
                    const double angle = (local_x - straight_before_arc_) / arc_radius_;
                    path_x = straight_before_arc_ + arc_radius_ * std::sin(angle);
                    local_y = arc_turn_sign_ * arc_radius_ * (1.0 - std::cos(angle));
                    local_heading = arc_turn_sign_ * angle;
                }
                else
                {
                    const double exit_distance = local_x - straight_before_arc_ - arc_length;
                    path_x = straight_before_arc_ + arc_radius_ * std::sin(arc_angle) +
                        exit_distance * std::cos(arc_angle);
                    local_y = arc_turn_sign_ * (
                        arc_radius_ * (1.0 - std::cos(arc_angle)) +
                        exit_distance * std::sin(arc_angle));
                    local_heading = arc_turn_sign_ * arc_angle;
                }
            }
            else if ((profile_ == "gentle_s" || profile_ == "s_curve") &&
                local_x >= lead_in_length_ &&
                local_x <= lead_in_length_ + s_curve_length_)
            {
                // sin^3 在接入和退出处的横向位移、切线都为零，避免折线突变。
                const double t = (local_x - lead_in_length_) / s_curve_length_;
                const double phase = 2.0 * kPi * t;
                const double s = std::sin(phase);
                local_y = s_amplitude_ * s * s * s;
                dy_dx = s_amplitude_ * 3.0 * s * s * std::cos(phase) *
                            (2.0 * kPi / s_curve_length_);
                local_heading = std::atan(dy_dx);
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = frame_id_;
            const double c = std::cos(origin_yaw_);
            const double s = std::sin(origin_yaw_);
            pose.pose.position.x = origin_x_ + c * path_x - s * local_y;
            pose.pose.position.y = origin_y_ + s * path_x + c * local_y;
            pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, origin_yaw_ + local_heading);
            pose.pose.orientation = tf2::toMsg(q);
            path_.poses.push_back(pose);
        }
    }

    void publish_path()
    {
        if (!anchored_)
        {
            return;
        }
        path_.header.stamp = now();
        for (auto &pose : path_.poses)
        {
            pose.header.stamp = path_.header.stamp;
        }
        path_pub_->publish(path_);
    }

    std::string profile_;
    std::string topic_;
    std::string frame_id_{"odom"};
    double sample_spacing_{};
    double straight_length_{};
    double lead_in_length_{};
    double s_curve_length_{};
    double s_amplitude_{};
    double straight_before_arc_{};
    double arc_radius_{};
    double arc_angle_degrees_{};
    double arc_exit_length_{};
    double arc_turn_sign_{};
    double publish_period_s_{};
    double origin_x_{};
    double origin_y_{};
    double origin_yaw_{};
    double path_length_{};
    bool anchored_{false};

    nav_msgs::msg::Path path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPathPublisher>());
    rclcpp::shutdown();
    return 0;
}
