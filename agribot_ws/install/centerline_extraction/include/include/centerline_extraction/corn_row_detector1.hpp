#ifndef CORN_ROW_DETECTOR_HPP
#define CORN_ROW_DETECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <deque>
#include <vector>

class CornRowDetector : public rclcpp::Node
{
public:
    CornRowDetector();

private:
    // 回调函数
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // 点云处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_plants(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> group_into_rows(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);
    
    // 中心线计算（限制前方2米直线拟合）
    nav_msgs::msg::Path compute_center_line(const std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>>& rows);
    std::vector<pcl::PointXYZ> get_all_points(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& row);
    pcl::PointXYZ get_row_centroid(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& row);
    
    // 直线拟合专用
    struct LineParam { float k; float b; };  // 直线参数：y = kx + b
    LineParam fit_linear(const std::vector<pcl::PointXYZ>& points);  // 一阶线性拟合
    float evaluate_line(const LineParam& line, float x);  // 计算直线上的y值
    
    // 平滑与优化
    nav_msgs::msg::Path smooth_path(const nav_msgs::msg::Path& raw_path);
    std::vector<geometry_msgs::msg::PoseStamped> remove_outliers(
        const std::vector<geometry_msgs::msg::PoseStamped>& points);
    std::vector<geometry_msgs::msg::PoseStamped> temporal_smoothing(
        const std::vector<geometry_msgs::msg::PoseStamped>& current_poses);

    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_pub_;

    // 检测参数
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    float row_distance_threshold_;
    float y_crop_min_;
    float y_crop_max_;
    float z_min_;
    float z_max_;
    float fit_distance_;  // 拟合距离（前方2米）

    // 平滑参数
    int temporal_window_size_;
    std::deque<std::vector<geometry_msgs::msg::PoseStamped>> path_buffer_;
};

#endif  // CORN_ROW_DETECTOR_HPP
