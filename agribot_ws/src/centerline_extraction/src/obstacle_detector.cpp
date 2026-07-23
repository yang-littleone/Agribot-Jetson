#include "centerline_extraction/obstacle_detector.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <cmath>

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector")
{
    // 声明参数
    declare_parameter("obstacle_detection_range", 2.0);
    declare_parameter("obstacle_min_height", 0.05);
    declare_parameter("obstacle_max_height", 1.0);
    declare_parameter("obstacle_safety_distance", 0.2);
    declare_parameter("path_width", 1.0);  // 路径宽度（从中心线向两边各path_width/2）
    
    // 获取参数
    get_parameter("obstacle_detection_range", obstacle_detection_range_);
    get_parameter("obstacle_min_height", obstacle_min_height_);
    get_parameter("obstacle_max_height", obstacle_max_height_);
    get_parameter("obstacle_safety_distance", obstacle_safety_distance_);
    get_parameter("path_width", path_width_);
    
    // 创建订阅者和发布者
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mid360_PointCloud2", 10,
        std::bind(&ObstacleDetector::pointCloudCallback, this, std::placeholders::_1)
    );
    
    obstacle_detected_pub_ = create_publisher<std_msgs::msg::Bool>("/obstacle_detected", 10);
    
    RCLCPP_INFO(get_logger(), "Obstacle detector initialized");
}

void ObstacleDetector::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // 检测障碍物
    bool obstacle_detected = detectObstacles(cloud);
    
    // 发布检测结果
    std_msgs::msg::Bool msg_out;
    msg_out.data = obstacle_detected;
    obstacle_detected_pub_->publish(msg_out);
}

bool ObstacleDetector::detectObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    int obstacle_points = 0;
    
    for (const auto& point : cloud->points)
    {
        // 检查点是否有效
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
            continue;
        
        // 检查是否在检测范围内
        if (point.x < 0 || point.x > obstacle_detection_range_)
            continue;
        
        // 检查是否在路径宽度范围内
        if (fabs(point.y) > path_width_ / 2.0)
            continue;
        
        // 检查高度是否在障碍物高度范围内
        if (point.z > obstacle_min_height_ && point.z < obstacle_max_height_)
        {
            // 检查距离是否小于安全距离
            float distance = sqrt(point.x*point.x + point.y*point.y);
            if (distance < obstacle_safety_distance_)
            {
                obstacle_points++;
            }
        }
    }
    
    // 如果检测到足够多的障碍物点，认为存在障碍物
    if (obstacle_points > 10)  // 阈值可调整
    {
        RCLCPP_WARN(get_logger(), "Obstacle detected! Points: %d", obstacle_points);
        return true;
    }
    
    return false;
}
