#ifndef CORN_ROW_DETECTOR_HPP
#define CORN_ROW_DETECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
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
    
    // 点云处理（修正函数声明，确保与实现一致）
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_plants(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> group_into_rows(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);
    
    // 中心线计算
    nav_msgs::msg::Path compute_center_line(const std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>>& rows);
    std::vector<pcl::PointXYZ> get_all_points(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& row);
    pcl::PointXYZ find_nearby_point(const std::vector<pcl::PointXYZ>& points, float x);
    pcl::PointXYZ get_row_centroid(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& row);
    
    // 高级平滑算法（核心）
    nav_msgs::msg::Path advanced_smoothing(const nav_msgs::msg::Path& raw_path);
    std::vector<geometry_msgs::msg::PoseStamped> spatial_smoothing(
        const std::vector<geometry_msgs::msg::PoseStamped>& poses);
    std::vector<geometry_msgs::msg::PoseStamped> temporal_smoothing(
        const std::vector<geometry_msgs::msg::PoseStamped>& current_poses);
    std::vector<geometry_msgs::msg::PoseStamped> remove_outliers(
        const std::vector<geometry_msgs::msg::PoseStamped>& points);
    bool is_jump(const geometry_msgs::msg::PoseStamped& prev, const geometry_msgs::msg::PoseStamped& curr);

    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_pub_;

    // 检测参数
    float cluster_tolerance_ = 0.2f;
    int min_cluster_size_ = 10;
    int max_cluster_size_ = 200;
    float row_distance_threshold_ = 0.5f;
    float y_crop_min_ = -2.0f;
    float y_crop_max_ = 2.0f;
    float z_min_ = 0.1f;
    float z_max_ = 1.0f;

    // 平滑专用参数（使用size_t避免类型不匹配）
    size_t time_window_size_;       // 改为size_t与deque.size()匹配
    float spatial_smooth_weight_;   // 空间平滑权重
    float max_jump_distance_;       // 最大允许跳变距离
    float outlier_std_threshold_;   // 异常值检测的标准差阈值
    int spline_segments_;           // 样条曲线分段数
    std::deque<std::vector<geometry_msgs::msg::PoseStamped>> path_buffer_;  // 路径缓存
};

#endif  // CORN_ROW_DETECTOR_HPP
    