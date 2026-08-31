#ifndef CORN_ROW_DETECTOR_PROJECTION_HPP
#define CORN_ROW_DETECTOR_PROJECTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pcl/point_cloud.h" // provide pcl point cloud type
#include "pcl/point_types.h" // provide pcl point types
#include <deque>
#include <string>
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_mode_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_row_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_row_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr left_boundary_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr right_boundary_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr corridor_width_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr corridor_safety_margin_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr corridor_error_budget_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr corridor_confidence_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr detection_diagnostics_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr performance_metrics_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr headland_detected_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    float z_min_ = 0.0;
    float z_max_ = 0.5;
    float voxel_size_ = 0.02;
    float x_min_ = 0.0;
    float x_max_ = 2.0;
    float y_min_ = -1.0;
    float y_max_ = 1.0;
    float path_length_ = 2.0;
    float path_step_ = 0.1;
    float min_row_separation_ = 0.3;
    float max_row_separation_ = 3.0;
    float max_line_slope_ = 5.0;
    float plants_ahead_x_ = 0.5;
    float robust_fit_residual_threshold_ = 0.12;
    float section_width_ = 0.25;
    int min_section_points_ = 3;
    float platform_width_ = 0.30;
    float desired_row_separation_ = 0.8;
    float segmented_center_weight_ = 0.0;
    float segmented_boundary_weight_ = 0.0;
    float innermost_bin_width_ = 0.20;
    float innermost_row_band_width_ = 0.18;
    float innermost_quantile_ = 0.20;
    int min_innermost_seeds_ = 4;
    float max_row_alignment_yaw_ = 0.75;
    float line_filter_alpha_ = 0.25;
    float max_line_lateral_jump_ = 0.18;
    float max_row_width_jump_ = 0.25;
    float max_row_yaw_jump_ = 0.45;
    int max_line_lost_frames_ = 30;
    float line_fit_x_min_ = 0.20;
    bool use_simple_inner_row_mode_ = true;
    bool use_constrained_row_pair_ = false;
    float row_histogram_bin_width_ = 0.025;
    float row_peak_half_width_ = 0.055;
    float row_yaw_search_step_ = 0.01;
    float row_pair_width_penalty_ = 80.0;
    float row_yaw_prior_tolerance_ = 0.15;
    float row_coverage_bin_width_ = 0.25;
    int row_coverage_min_points_ = 3;
    float row_min_coverage_ratio_ = 0.35;
    float row_min_longitudinal_span_ = 1.0;
    float row_coverage_score_weight_ = 50.0;
    bool use_row_yaw_estimation_ = true;
    bool use_parallel_row_model_ = true;
    bool enable_innermost_row_extraction_ = true;
    bool enable_robust_refinement_ = true;
    bool enable_temporal_tracking_ = true;
    bool enable_quality_evaluation_ = true;
    float plant_safety_clearance_ = 0.10;
    float quality_support_weight_ = 0.30;
    float quality_observation_weight_ = 0.20;
    float quality_width_weight_ = 0.20;
    float quality_residual_weight_ = 0.15;
    float quality_safety_weight_ = 0.15;
    float quality_observation_fallback_score_ = 0.45;
    std::string base_frame_ = "base_link";
    std::string output_frame_ = "odom";

    // 多行场景下选择最内侧行的参数
    float lateral_cluster_eps_ = 0.2; // 横向聚类阈值（米）
    int min_cluster_size_ = 10;       // 单个簇的最小点数
    // 自适应阈值参数：若启用，将根据点列间距的中位 gap 自适应分簇
    bool adaptive_lateral_threshold_ = false; // 是否启用自适应阈值（median-gap）
    float gap_multiplier_ = 2.0;              // 中位 gap 的乘数，作为分割阈值的放大因子
    bool enable_headland_detection_ = true;
    float headland_low_confidence_threshold_ = 0.35;
    int headland_min_side_points_ = 80;
    int headland_min_path_points_ = 5;
    int headland_candidate_frames_ = 6;
    int headland_candidate_count_ = 0;

    // 平滑处理参数
    int time_window_size_ = 1;          // time window size
    float spatial_smooth_weight_ = 0.7; // spatial smooth weight
    float outlier_threshold_ = 2.0;     // outlier threshold
    // record path history
    std::deque<nav_msgs::msg::Path> path_history_;
    bool has_tracked_lines_ = false;
    bool has_tracked_row_yaw_ = false;
    bool has_tracked_global_row_yaw_ = false;
    bool has_navigation_mode_ = false;
    int line_lost_count_ = 0;
    float tracked_row_yaw_ = 0.0;
    float tracked_global_row_yaw_ = 0.0;
    std::string last_navigation_mode_;
    std::pair<float, float> tracked_left_line_{0.0f, 0.0f};
    std::pair<float, float> tracked_right_line_{0.0f, 0.0f};
    float last_corridor_width_ = 0.0f;
    float last_corridor_safety_margin_ = 0.0f;
    float last_corridor_error_budget_ = 0.0f;
    float last_corridor_confidence_ = 0.0f;
    float last_raw_corridor_confidence_ = 0.0f;
    float last_quality_support_score_ = 0.0f;
    float last_quality_observation_score_ = 0.0f;
    float last_quality_width_score_ = 0.0f;
    float last_quality_residual_score_ = 0.0f;
    float last_quality_safety_score_ = 0.0f;
    float last_quality_observed_valid_ratio_ = 0.0f;
    float last_left_longitudinal_coverage_ = 0.0f;
    float last_right_longitudinal_coverage_ = 0.0f;
    bool enable_performance_metrics_ = false;

public:
    CornRowDetectorProjection();

private:
    // the callback of point cloud subscriber
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void navigation_mode_callback(const std_msgs::msg::String::SharedPtr msg);

    // preprocess point cloud
    PointCloudXYZPtr preprocess_point_cloud(PointCloudXYZPtr input_cloud);

    // transform point cloud into the robot base frame before local row extraction
    PointCloudXYZPtr transform_cloud_to_base_frame(PointCloudXYZPtr input_cloud, const std_msgs::msg::Header &header);

    // projection point cloud to xy plane
    PointCloudXYZPtr projection_point_cloud(PointCloudXYZPtr input_cloud);

    float estimate_row_yaw(PointCloudXYZPtr cloud);
    float filter_global_row_yaw(float measured_global_row_yaw);
    float estimate_global_row_yaw(
        PointCloudXYZPtr cloud_base,
        const geometry_msgs::msg::TransformStamped &output_from_base);
    bool lookup_output_from_base_transform(geometry_msgs::msg::TransformStamped &output_from_base);
    float normalize_angle(float angle) const;
    float normalize_axis_angle(float angle) const;
    void reset_tracking_state(
        const std::string &reason,
        bool preserve_global_row_yaw = false);
    PointCloudXYZPtr rotate_cloud_to_row_frame(PointCloudXYZPtr cloud, float row_yaw);
    PointCloudXYZPtr rotate_cloud_to_base_frame(PointCloudXYZPtr cloud, float row_yaw);

    // split point cloud to left and right rows
    std::pair<PointCloudXYZPtr, PointCloudXYZPtr> split_left_right_rows(PointCloudXYZPtr input_cloud);

    // extract innermost row from side cloud (when multiple parallel rows exist)
    PointCloudXYZPtr extract_innermost_row(PointCloudXYZPtr cloud, bool left);

    // Jointly find the nearest left/right density peaks under row-spacing constraints.
    bool extract_constrained_row_pair(
        PointCloudXYZPtr cloud,
        float search_center_yaw,
        float search_half_range,
        float &row_yaw,
        PointCloudXYZPtr &left_row,
        PointCloudXYZPtr &right_row);

    // fit line for point cloud
    std::pair<float, float> fit_line(PointCloudXYZPtr cloud);

    float filter_row_yaw(float measured_row_yaw);
    bool update_tracked_lines(
        const std::pair<float, float> &measured_left_line,
        const std::pair<float, float> &measured_right_line,
        float measured_row_yaw,
        std::pair<float, float> &tracked_left_line,
        std::pair<float, float> &tracked_right_line,
        float &tracked_row_yaw);

    // create segmented under-canopy traversable corridor from row boundary observations
    nav_msgs::msg::Path create_corridor_path(
        PointCloudXYZPtr left_cloud,
        PointCloudXYZPtr right_cloud,
        const std::pair<float, float> &left_line,
        const std::pair<float, float> &right_line,
        float row_yaw,
        const std_msgs::msg::Header &header);

    nav_msgs::msg::Path transform_path_to_output_frame(const nav_msgs::msg::Path &path_base_link);

    bool estimate_lateral_median(
        PointCloudXYZPtr cloud,
        float x,
        float fallback_y,
        float &estimated_y,
        int &support_count);

    void publish_corridor_metrics(
        float width,
        float safety_margin,
        float error_budget,
        float confidence);
    void publish_detection_diagnostics(
        bool valid,
        int left_points,
        int right_points,
        float row_yaw,
        float center_offset,
        const std::pair<float, float> &left_line,
        const std::pair<float, float> &right_line,
        int path_points);
    void publish_headland_detection(
        bool valid,
        int left_points,
        int right_points,
        int path_points);

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
};

#endif
