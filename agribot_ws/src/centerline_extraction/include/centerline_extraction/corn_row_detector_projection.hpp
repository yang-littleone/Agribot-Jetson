#ifndef CORN_ROW_DETECTOR_PROJECTION_HPP
#define CORN_ROW_DETECTOR_PROJECTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h" // provide pcl point cloud type
#include "pcl/point_types.h" // provide pcl point types
#include <deque>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// define point cloud type
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class CornRowDetectorProjection : public rclcpp::Node
{
private:
    // define subscriber and publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_row_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_row_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // 订阅小车里程计
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_viz_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double robot_current_x_ = 0.0; // 存储小车当前x坐标
    double robot_current_y_ = 0.0; // （可选）存储y坐标，备用
    float z_min_ = 0.0;
    float z_max_ = 0.5;
    float voxel_size_ = 0.02;

    // 多行场景下选择最内侧行的参数
    float lateral_cluster_eps_ = 0.2; // 横向聚类阈值（米）
    int min_cluster_size_ = 10;       // 单个簇的最小点数
    // 自适应阈值参数：若启用，将根据点列间距的中位 gap 自适应分簇
    bool adaptive_lateral_threshold_ = false; // 是否启用自适应阈值（median-gap）
    float gap_multiplier_ = 2.0;              // 中位 gap 的乘数，作为分割阈值的放大因子

    // 平滑处理参数
    int time_window_size_ = 5;          // time window size
    float spatial_smooth_weight_ = 0.7; // spatial smooth weight
    float outlier_threshold_ = 2.0;     // outlier threshold
    float ceneterline_length_;
    float path_step_;
    // record path history
    std::deque<nav_msgs::msg::Path> path_history_;

public:
    CornRowDetectorProjection();

private:
    // the callback of point cloud subscriber
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // preprocess point cloud
    PointCloudXYZPtr preprocess_point_cloud(PointCloudXYZPtr input_cloud);

    // projection point cloud to xy plane
    PointCloudXYZPtr projection_point_cloud(PointCloudXYZPtr input_cloud);

    // split point cloud to left and right rows
    std::pair<PointCloudXYZPtr, PointCloudXYZPtr> split_left_right_rows(PointCloudXYZPtr input_cloud);

    // extract innermost row from side cloud (when multiple parallel rows exist)
    PointCloudXYZPtr extract_innermost_row(PointCloudXYZPtr cloud, bool left);

    // fit line for point cloud
    std::pair<float, float> fit_line(PointCloudXYZPtr cloud);

    // create path from line parameters
    nav_msgs::msg::Path create_path(float slope, float intercept, const std_msgs::msg::Header &header);

    // 平滑处理函数
    nav_msgs::msg::Path smooth_path(const nav_msgs::msg::Path &raw_path);

    // 异常值剔除
    nav_msgs::msg::Path remove_outliers(const nav_msgs::msg::Path &path);

    // 空间平滑
    nav_msgs::msg::Path spatial_smoothing(const nav_msgs::msg::Path &path);

    // 时间平滑
    nav_msgs::msg::Path temporal_smoothing(const nav_msgs::msg::Path &path);

    // 发布空路径以清空显示
    void publish_empty_path(const std_msgs::msg::Header &header);

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif