#ifndef OBSTACLE_DETECTOR_HPP
#define OBSTACLE_DETECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_types.h"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <pcl/point_cloud.h>  // 添加必要的PCL头文件
#include <pcl/point_types.h>  // 添加必要的PCL头文件

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector();
    
private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    bool detectObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_pub_;
    
    // 障碍物检测参数
    float obstacle_detection_range_;  // 检测范围（米）
    float obstacle_min_height_;       // 障碍物最小高度（米）
    float obstacle_max_height_;       // 障碍物最大高度（米）
    float obstacle_safety_distance_;  // 安全距离（米）
    float path_width_;                // 路径宽度（米）
};

#endif // OBSTACLE_DETECTOR_HPP
