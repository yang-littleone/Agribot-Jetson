#ifndef CENTERLINE_EXTRACTION_CORN_ROW_DETECTOR_PROJECTION_HPP_
#define CENTERLINE_EXTRACTION_CORN_ROW_DETECTOR_PROJECTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <deque>

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class CornRowDetectorProjection : public rclcpp::Node
{
public:
    CornRowDetectorProjection();
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
    // 预处理
    PointCloudXYZPtr preprocess_point_cloud(PointCloudXYZPtr input_cloud);
    PointCloudXYZPtr projection_point_cloud(PointCloudXYZPtr input_cloud);
    std::pair<PointCloudXYZPtr, PointCloudXYZPtr> split_left_right_rows(PointCloudXYZPtr input_cloud);
    std::pair<float, float> fit_line(PointCloudXYZPtr cloud);
    
    // 路径生成与平滑
    nav_msgs::msg::Path create_path(float slope, float intercept, const std_msgs::msg::Header &header);
    nav_msgs::msg::Path smooth_path(const nav_msgs::msg::Path &raw_path);
    nav_msgs::msg::Path remove_outliers(const nav_msgs::msg::Path &path);
    nav_msgs::msg::Path spatial_smoothing(const nav_msgs::msg::Path &path);
    nav_msgs::msg::Path temporal_smoothing(const nav_msgs::msg::Path &path);
    
    // 辅助函数
    void publish_empty_path(const std_msgs::msg::Header &header);
    void publish_point_clouds(PointCloudXYZPtr proj_cloud, PointCloudXYZPtr left_cloud, PointCloudXYZPtr right_cloud, const std_msgs::msg::Header &header);

    // 订阅者/发布者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_row_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_row_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_viz_pub_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 参数
    float z_min_, z_max_;
    float voxel_size_;
    int time_window_size_;
    float spatial_smooth_weight_;
    float outlier_threshold_;
    float ceneterline_length_;
    float path_step_;
    float max_slope_;

    // 状态
    float robot_current_x_, robot_current_y_;
    std::deque<nav_msgs::msg::Path> path_history_;
};

#endif // CENTERLINE_EXTRACTION_CORN_ROW_DETECTOR_PROJECTION_HPP_