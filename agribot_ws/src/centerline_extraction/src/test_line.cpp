#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * @brief 测试曲线生成节点
 * 
 * 该节点发布一条固定的圆弧曲线，模拟玉米行中心线检测结果
 * 用于测试 Pure Pursuit 路径跟踪算法的准确性
 * 
 * 发布的圆弧特性：
 * - 半径：1.0 米
 * - 圆心：(0, 1.0)（在 odom 坐标系下）
 * - 起点：(0, 0)（odom 原点）
 * - 范围：从起点向前延伸 3.0 米
 * - 坐标系：odom_combined
 */
class TestLineGenerator : public rclcpp::Node
{
public:
    TestLineGenerator() : Node("test_line_generator")
    {
        // 创建发布者
        center_line_pub_ = this->create_publisher<nav_msgs::msg::Path>("corn_row_center_line", 10);
        center_line_viz_pub_ = this->create_publisher<nav_msgs::msg::Path>("corn_row_center_line_viz", 10);
        
        // 声明参数
        this->declare_parameter<float>("arc_radius", -1.0f);       // 圆弧半径
        this->declare_parameter<float>("arc_length", 3.0f);       // 圆弧长度
        this->declare_parameter<int>("publish_rate", 10);         // 发布频率 (Hz)
        
        // 获取参数
        this->get_parameter("arc_radius", arc_radius_);
        this->get_parameter("arc_length", arc_length_);
        this->get_parameter("publish_rate", publish_rate_);
        
        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&TestLineGenerator::publish_test_line, this));
        
        RCLCPP_INFO(this->get_logger(), "Test Arc Generator initialized");
        RCLCPP_INFO(this->get_logger(), "Arc parameters: radius=%.2f, length=%.2f", 
                    arc_radius_, arc_length_);
        RCLCPP_INFO(this->get_logger(), "Publish rate: %dHz", publish_rate_);
    }

private:
    void publish_test_line()
    {
        // 创建路径消息
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "odom_combined";
        path_msg.header.stamp = this->now();
        
        // 计算圆弧参数
        // 圆心在 (0, arc_radius)，起点在 (0, 0)
        float center_x = 0.0f;
        float center_y = arc_radius_;
        
        // 计算圆弧对应的角度范围
        // 弧长 = 半径 × 角度（弧度）
        float total_angle = arc_length_ / arc_radius_;
        
        // 点间距（沿圆弧）
        float point_spacing_along_arc = 0.1f;
        int num_points = static_cast<int>(arc_length_ / point_spacing_along_arc) + 1;
        
        RCLCPP_DEBUG(this->get_logger(), "Generating arc with %d points, total angle=%.2f rad", 
                     num_points, total_angle);
        
        // 生成圆弧路径点
        for (int i = 0; i <= num_points; ++i)
        {
            // 当前弧长
            float current_arc_length = i * point_spacing_along_arc;
            
            // 当前角度（从 -π/2 开始，顺时针方向）
            float current_angle = -M_PI_2 + (current_arc_length / arc_radius_);
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            
            // 计算圆上的位置
            pose.pose.position.x = center_x + arc_radius_ * std::cos(current_angle);
            pose.pose.position.y = center_y + arc_radius_ * std::sin(current_angle);
            pose.pose.position.z = 0.0;
            
            // 设置朝向（切线方向）
            // 切线方向垂直于半径方向，加 π/2
            float yaw = current_angle + M_PI_2;
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation = tf2::toMsg(q);
            
            path_msg.poses.push_back(pose);
        }
        
        // 发布主中心线
        center_line_pub_->publish(path_msg);
        
        // 发布可视化路径（相同内容）
        nav_msgs::msg::Path viz_path = path_msg;
        viz_path.header.frame_id = "base_link";
        center_line_viz_pub_->publish(viz_path);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), 
                             *this->get_clock(), 
                             1000, 
                             "Published arc: %zu points, radius=%.3f, length=%.3f",
                             path_msg.poses.size(), 
                             arc_radius_, 
                             arc_length_);
    }
    
    // 成员变量
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_viz_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    float arc_radius_;
    float arc_length_;
    int publish_rate_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TestLineGenerator>();
    
    RCLCPP_INFO(node->get_logger(), "Starting test arc generator...");
    RCLCPP_INFO(node->get_logger(), "This will publish a simulated corn row center line (arc curve).");
    RCLCPP_INFO(node->get_logger(), "You can adjust parameters via ROS 2 parameter server.");
    RCLCPP_INFO(node->get_logger(), "\nExample parameter commands:");
    RCLCPP_INFO(node->get_logger(), "  ros2 param set /test_line_generator arc_radius 1.5");
    RCLCPP_INFO(node->get_logger(), "  ros2 param set /test_line_generator arc_length 4.0");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
