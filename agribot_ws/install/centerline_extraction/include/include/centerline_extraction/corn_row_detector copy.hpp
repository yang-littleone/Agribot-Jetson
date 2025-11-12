#ifndef CORN_ROW_DETECTOR_HPP
#define CORN_ROW_DETECTOR_HPP 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/common/centroid.h"

class CornRowDetector : public rclcpp::Node
{
public:
    CornRowDetector();
    

private:
    void get_parameters();

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    // 检测左右两行玉米
    std::pair<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>> 
    detect_left_right_rows(const std::vector<Eigen::Vector3f>& centers);
    // 计算两行之间的中心线
    std::vector<Eigen::Vector3f> compute_center_line(
        const std::vector<Eigen::Vector3f>& left_points,
        const std::vector<Eigen::Vector3f>& right_points);
    
    // 发布中心线
    void publish_center_line(const std::vector<Eigen::Vector3f>& center_line, 
                           const std_msgs::msg::Header& header);

    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_pub_;
    
    // 算法参数
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    float max_distance_;
    float row_width_min_;
    float row_width_max_;
};

#endif