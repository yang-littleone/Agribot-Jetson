#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"                 // provide pcl point cloud type
#include "pcl/point_types.h"                 // provide pcl point types
#include <pcl_conversions/pcl_conversions.h> // provite ros2 to pcl conversion functions
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <cmath>
#include <numeric>
#include <vector>
#include <algorithm>
#include <limits>
#include "centerline_extraction/corn_row_detector_projection.hpp"

// define point cloud type
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

CornRowDetectorProjection::CornRowDetectorProjection() : Node("corn_row_detector_projection")
{
    const auto point_cloud_topic =
        this->declare_parameter<std::string>("point_cloud_topic", "/livox/lidar");
    // create subscribers and publishers
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic, 10, std::bind(&CornRowDetectorProjection::point_cloud_callback, this, std::placeholders::_1));
    navigation_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/navigation_mode", 10,
        std::bind(&CornRowDetectorProjection::navigation_mode_callback, this, std::placeholders::_1));
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_projected", 10);
    left_row_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("left_row_points", 10);
    right_row_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("right_row_points", 10);
    center_line_pub_ = this->create_publisher<nav_msgs::msg::Path>("corn_row_center_line", 10);
    center_line_viz_pub_ = this->create_publisher<nav_msgs::msg::Path>("corn_row_center_line_viz", 10);
    left_boundary_pub_ = this->create_publisher<nav_msgs::msg::Path>("under_canopy_left_boundary", 10);
    right_boundary_pub_ = this->create_publisher<nav_msgs::msg::Path>("under_canopy_right_boundary", 10);
    corridor_width_pub_ = this->create_publisher<std_msgs::msg::Float32>("corridor_width", 10);
    corridor_safety_margin_pub_ = this->create_publisher<std_msgs::msg::Float32>("corridor_safety_margin", 10);
    corridor_error_budget_pub_ = this->create_publisher<std_msgs::msg::Float32>("corridor_error_budget", 10);
    corridor_confidence_pub_ = this->create_publisher<std_msgs::msg::Float32>("corridor_confidence", 10);
    detection_diagnostics_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("centerline_detection_diagnostics", 10);
    headland_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("headland_detected", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // declare parameters
    this->declare_parameter<float>("z_min", 0.0);
    this->declare_parameter<float>("z_max", 0.5);
    this->declare_parameter<float>("voxel_size", 0.02);
    this->declare_parameter<float>("x_min", 0.0);
    this->declare_parameter<float>("x_max", 2.0);
    this->declare_parameter<float>("y_min", -1.0);
    this->declare_parameter<float>("y_max", 1.0);
    this->declare_parameter<float>("path_length", 2.0);
    this->declare_parameter<float>("path_step", 0.1);
    this->declare_parameter<float>("min_row_separation", 0.3);
    this->declare_parameter<float>("max_row_separation", 3.0);
    this->declare_parameter<float>("max_line_slope", 5.0);
    this->declare_parameter<float>("plants_ahead_x", 0.5);
    this->declare_parameter<float>("robust_fit_residual_threshold", 0.12);
    this->declare_parameter<float>("section_width", 0.25);
    this->declare_parameter<int>("min_section_points", 3);
    this->declare_parameter<float>("platform_width", 0.30);
    this->declare_parameter<float>("desired_row_separation", 0.8);
    this->declare_parameter<float>("segmented_center_weight", 0.0);
    this->declare_parameter<float>("segmented_boundary_weight", 0.0);
    this->declare_parameter<float>("innermost_bin_width", 0.20);
    this->declare_parameter<float>("innermost_row_band_width", 0.18);
    this->declare_parameter<float>("innermost_quantile", 0.20);
    this->declare_parameter<int>("min_innermost_seeds", 4);
    this->declare_parameter<float>("max_row_alignment_yaw", 0.75);
    this->declare_parameter<float>("line_filter_alpha", 0.25);
    this->declare_parameter<float>("max_line_lateral_jump", 0.18);
    this->declare_parameter<float>("max_row_width_jump", 0.25);
    this->declare_parameter<float>("max_row_yaw_jump", 0.45);
    this->declare_parameter<int>("max_line_lost_frames", 30);
    this->declare_parameter<float>("line_fit_x_min", 0.20);
    this->declare_parameter<bool>("use_simple_inner_row_mode", true);
    this->declare_parameter<bool>("use_row_yaw_estimation", true);
    this->declare_parameter<bool>("use_parallel_row_model", true);
    this->declare_parameter<bool>("enable_innermost_row_extraction", true);
    this->declare_parameter<bool>("enable_robust_refinement", true);
    this->declare_parameter<bool>("enable_temporal_tracking", true);
    this->declare_parameter<bool>("enable_quality_evaluation", true);
    this->declare_parameter<float>("plant_safety_clearance", 0.10);
    this->declare_parameter<float>("quality_support_weight", 0.30);
    this->declare_parameter<float>("quality_observation_weight", 0.20);
    this->declare_parameter<float>("quality_width_weight", 0.20);
    this->declare_parameter<float>("quality_residual_weight", 0.15);
    this->declare_parameter<float>("quality_safety_weight", 0.15);
    this->declare_parameter<float>("quality_observation_fallback_score", 0.45);
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("output_frame", "odom");
    this->declare_parameter<int>("time_window_size", 1);
    this->declare_parameter<float>("spatial_smooth_weight", 0.7);
    this->declare_parameter<float>("outlier_threshold", 2.0);
    this->declare_parameter<float>("lateral_cluster_eps", 0.1);
    this->declare_parameter<int>("min_cluster_size", 10);
    this->declare_parameter<bool>("adaptive_lateral_threshold", false);
    this->declare_parameter<float>("gap_multiplier", 2.0);
    this->declare_parameter<bool>("enable_headland_detection", true);
    this->declare_parameter<float>("headland_low_confidence_threshold", 0.35);
    this->declare_parameter<int>("headland_min_side_points", 80);
    this->declare_parameter<int>("headland_min_path_points", 5);
    this->declare_parameter<int>("headland_candidate_frames", 6);

    // get parameters
    this->get_parameter("z_min", z_min_);
    this->get_parameter("z_max", z_max_);
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("x_min", x_min_);
    this->get_parameter("x_max", x_max_);
    this->get_parameter("y_min", y_min_);
    this->get_parameter("y_max", y_max_);
    this->get_parameter("path_length", path_length_);
    this->get_parameter("path_step", path_step_);
    this->get_parameter("min_row_separation", min_row_separation_);
    this->get_parameter("max_row_separation", max_row_separation_);
    this->get_parameter("max_line_slope", max_line_slope_);
    this->get_parameter("plants_ahead_x", plants_ahead_x_);
    this->get_parameter("robust_fit_residual_threshold", robust_fit_residual_threshold_);
    this->get_parameter("section_width", section_width_);
    this->get_parameter("min_section_points", min_section_points_);
    this->get_parameter("platform_width", platform_width_);
    this->get_parameter("desired_row_separation", desired_row_separation_);
    this->get_parameter("segmented_center_weight", segmented_center_weight_);
    this->get_parameter("segmented_boundary_weight", segmented_boundary_weight_);
    this->get_parameter("innermost_bin_width", innermost_bin_width_);
    this->get_parameter("innermost_row_band_width", innermost_row_band_width_);
    this->get_parameter("innermost_quantile", innermost_quantile_);
    this->get_parameter("min_innermost_seeds", min_innermost_seeds_);
    this->get_parameter("max_row_alignment_yaw", max_row_alignment_yaw_);
    this->get_parameter("line_filter_alpha", line_filter_alpha_);
    this->get_parameter("max_line_lateral_jump", max_line_lateral_jump_);
    this->get_parameter("max_row_width_jump", max_row_width_jump_);
    this->get_parameter("max_row_yaw_jump", max_row_yaw_jump_);
    this->get_parameter("max_line_lost_frames", max_line_lost_frames_);
    this->get_parameter("line_fit_x_min", line_fit_x_min_);
    this->get_parameter("use_simple_inner_row_mode", use_simple_inner_row_mode_);
    this->get_parameter("use_row_yaw_estimation", use_row_yaw_estimation_);
    this->get_parameter("use_parallel_row_model", use_parallel_row_model_);
    this->get_parameter("enable_innermost_row_extraction", enable_innermost_row_extraction_);
    this->get_parameter("enable_robust_refinement", enable_robust_refinement_);
    this->get_parameter("enable_temporal_tracking", enable_temporal_tracking_);
    this->get_parameter("enable_quality_evaluation", enable_quality_evaluation_);
    this->get_parameter("plant_safety_clearance", plant_safety_clearance_);
    this->get_parameter("quality_support_weight", quality_support_weight_);
    this->get_parameter("quality_observation_weight", quality_observation_weight_);
    this->get_parameter("quality_width_weight", quality_width_weight_);
    this->get_parameter("quality_residual_weight", quality_residual_weight_);
    this->get_parameter("quality_safety_weight", quality_safety_weight_);
    this->get_parameter("quality_observation_fallback_score", quality_observation_fallback_score_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("output_frame", output_frame_);
    this->get_parameter("time_window_size", time_window_size_);
    this->get_parameter("spatial_smooth_weight", spatial_smooth_weight_);
    this->get_parameter("outlier_threshold", outlier_threshold_);
    this->get_parameter("lateral_cluster_eps", lateral_cluster_eps_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("adaptive_lateral_threshold", adaptive_lateral_threshold_);
    this->get_parameter("gap_multiplier", gap_multiplier_);
    this->get_parameter("enable_headland_detection", enable_headland_detection_);
    this->get_parameter("headland_low_confidence_threshold", headland_low_confidence_threshold_);
    this->get_parameter("headland_min_side_points", headland_min_side_points_);
    this->get_parameter("headland_min_path_points", headland_min_path_points_);
    this->get_parameter("headland_candidate_frames", headland_candidate_frames_);

    quality_support_weight_ = std::max(0.0f, quality_support_weight_);
    quality_observation_weight_ = std::max(0.0f, quality_observation_weight_);
    quality_width_weight_ = std::max(0.0f, quality_width_weight_);
    quality_residual_weight_ = std::max(0.0f, quality_residual_weight_);
    quality_safety_weight_ = std::max(0.0f, quality_safety_weight_);
    const float quality_weight_sum =
        quality_support_weight_ + quality_observation_weight_ +
        quality_width_weight_ + quality_residual_weight_ +
        quality_safety_weight_;
    if (quality_weight_sum < 1e-6f)
    {
        quality_support_weight_ = 0.30f;
        quality_observation_weight_ = 0.20f;
        quality_width_weight_ = 0.20f;
        quality_residual_weight_ = 0.15f;
        quality_safety_weight_ = 0.15f;
        RCLCPP_WARN(this->get_logger(), "All quality weights were zero; restored defaults");
    }
    else
    {
        quality_support_weight_ /= quality_weight_sum;
        quality_observation_weight_ /= quality_weight_sum;
        quality_width_weight_ /= quality_weight_sum;
        quality_residual_weight_ /= quality_weight_sum;
        quality_safety_weight_ /= quality_weight_sum;
    }
    quality_observation_fallback_score_ =
        std::clamp(quality_observation_fallback_score_, 0.0f, 1.0f);
    plant_safety_clearance_ = std::max(0.0f, plant_safety_clearance_);

    if (path_step_ <= 0.0f)
    {
        RCLCPP_WARN(this->get_logger(), "path_step must be positive; reset to 0.1");
        path_step_ = 0.1f;
    }
    if (path_length_ <= 0.0f)
    {
        RCLCPP_WARN(this->get_logger(), "path_length must be positive; reset to 2.0");
        path_length_ = 2.0f;
    }
    if (x_max_ <= x_min_)
    {
        RCLCPP_WARN(this->get_logger(), "x_max must be greater than x_min; reset ROI x range to [0.0, 2.0]");
        x_min_ = 0.0f;
        x_max_ = 2.0f;
    }
    if (y_max_ <= y_min_)
    {
        RCLCPP_WARN(this->get_logger(), "y_max must be greater than y_min; reset ROI y range to [-1.0, 1.0]");
        y_min_ = -1.0f;
        y_max_ = 1.0f;
    }
    if (section_width_ <= 0.0f)
    {
        RCLCPP_WARN(this->get_logger(), "section_width must be positive; reset to 0.25");
        section_width_ = 0.25f;
    }
    if (min_section_points_ < 1)
    {
        RCLCPP_WARN(this->get_logger(), "min_section_points must be at least 1; reset to 3");
        min_section_points_ = 3;
    }
    if (desired_row_separation_ <= 0.0f)
    {
        desired_row_separation_ = (min_row_separation_ + max_row_separation_) * 0.5f;
    }
    segmented_center_weight_ = std::clamp(segmented_center_weight_, 0.0f, 1.0f);
    segmented_boundary_weight_ = std::clamp(segmented_boundary_weight_, 0.0f, 1.0f);
    if (innermost_bin_width_ <= 0.0f)
    {
        RCLCPP_WARN(this->get_logger(), "innermost_bin_width must be positive; reset to 0.20");
        innermost_bin_width_ = 0.20f;
    }
    if (innermost_row_band_width_ <= 0.0f)
    {
        RCLCPP_WARN(this->get_logger(), "innermost_row_band_width must be positive; reset to 0.18");
        innermost_row_band_width_ = 0.18f;
    }
    innermost_quantile_ = std::clamp(innermost_quantile_, 0.0f, 0.5f);
    if (min_innermost_seeds_ < 2)
    {
        min_innermost_seeds_ = 2;
    }
    if (max_row_alignment_yaw_ <= 0.0f)
    {
        max_row_alignment_yaw_ = 0.75f;
    }
    line_filter_alpha_ = std::clamp(line_filter_alpha_, 0.0f, 1.0f);
    if (max_line_lateral_jump_ <= 0.0f)
    {
        max_line_lateral_jump_ = 0.18f;
    }
    if (max_row_width_jump_ <= 0.0f)
    {
        max_row_width_jump_ = 0.25f;
    }
    if (max_row_yaw_jump_ <= 0.0f)
    {
        max_row_yaw_jump_ = 0.45f;
    }
    if (max_line_lost_frames_ < 0)
    {
        max_line_lost_frames_ = 30;
    }
    if (line_fit_x_min_ < 0.0f)
    {
        line_fit_x_min_ = 0.0f;
    }
    headland_low_confidence_threshold_ = std::clamp(headland_low_confidence_threshold_, 0.0f, 1.0f);
    if (headland_min_side_points_ < 0)
    {
        headland_min_side_points_ = 0;
    }
    if (headland_min_path_points_ < 0)
    {
        headland_min_path_points_ = 0;
    }
    if (headland_candidate_frames_ < 1)
    {
        headland_candidate_frames_ = 1;
    }
}

// callback function
void CornRowDetectorProjection::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // convert pointcloud2 to pcl
    PointCloudXYZPtr cloud(new PointCloudXYZ());
    pcl::fromROSMsg(*msg, *cloud);

    PointCloudXYZPtr base_cloud = this->transform_cloud_to_base_frame(cloud, msg->header);
    std_msgs::msg::Header base_header = msg->header;
    base_header.frame_id = base_frame_;

    // preprocess point cloud in base frame
    PointCloudXYZPtr preprocessed_cloud = this->preprocess_point_cloud(base_cloud);

    // projection point cloud
    PointCloudXYZPtr projection_cloud = this->projection_point_cloud(preprocessed_cloud);
    float row_yaw = 0.0f;
    if (use_row_yaw_estimation_)
    {
        geometry_msgs::msg::TransformStamped output_from_base;
        if (this->lookup_output_from_base_transform(output_from_base))
        {
            const float global_row_yaw = this->filter_global_row_yaw(
                this->estimate_global_row_yaw(projection_cloud, output_from_base));
            const tf2::Quaternion q(
                output_from_base.transform.rotation.x,
                output_from_base.transform.rotation.y,
                output_from_base.transform.rotation.z,
                output_from_base.transform.rotation.w);
            double roll = 0.0;
            double pitch = 0.0;
            double robot_yaw = 0.0;
            tf2::Matrix3x3(q).getRPY(roll, pitch, robot_yaw);
            row_yaw = this->normalize_axis_angle(global_row_yaw - static_cast<float>(robot_yaw));
        }
        else
        {
            row_yaw = this->filter_row_yaw(this->estimate_row_yaw(projection_cloud));
        }
    }
    else
    {
        has_tracked_row_yaw_ = true;
        tracked_row_yaw_ = 0.0f;
    }
    PointCloudXYZPtr row_frame_cloud = (std::abs(row_yaw) > 1e-5f)
                                           ? this->rotate_cloud_to_row_frame(projection_cloud, row_yaw)
                                           : projection_cloud;

    // split point cloud to left and right rows
    auto [left_row_cloud, right_row_cloud] = this->split_left_right_rows(row_frame_cloud);

    // 从每侧点集合中提取最内侧一行（用于多行场景）
    PointCloudXYZPtr left_inner = enable_innermost_row_extraction_
                                      ? this->extract_innermost_row(left_row_cloud, true)
                                      : left_row_cloud;
    PointCloudXYZPtr right_inner = enable_innermost_row_extraction_
                                       ? this->extract_innermost_row(right_row_cloud, false)
                                       : right_row_cloud;

    // 如果提取后点数过少，复杂模式下可回退为整侧点；简单模式下不能回退，否则会混入外侧行。
    if (left_inner->size() < static_cast<size_t>(min_cluster_size_))
    {
        if (use_simple_inner_row_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "Left inner row too small (%ld) in simple mode", left_inner->size());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Left inner cluster too small (%ld), falling back to full left cloud", left_inner->size());
            left_inner = left_row_cloud;
        }
    }
    if (right_inner->size() < static_cast<size_t>(min_cluster_size_))
    {
        if (use_simple_inner_row_mode_)
        {
            RCLCPP_WARN(this->get_logger(), "Right inner row too small (%ld) in simple mode", right_inner->size());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Right inner cluster too small (%ld), falling back to full right cloud", right_inner->size());
            right_inner = right_row_cloud;
        }
    }

    // Check if we have enough points to generate a meaningful center line
    // Only proceed if both sides have sufficient points
    if (left_inner->size() < static_cast<size_t>(min_cluster_size_) || right_inner->size() < static_cast<size_t>(min_cluster_size_))
    {
        RCLCPP_WARN(this->get_logger(), "Insufficient points for center line generation. Left points: %ld, Right points: %ld",
                    left_inner->size(), right_inner->size());
        // 发布空路径以清空显示
        publish_empty_path(base_header);
        return;
    }

    // Check if there are plants in front of the robot (x > 0.5) using the inner rows
    bool has_plants_ahead = false;
    for (const auto &point : left_inner->points)
    {
        if (point.x > plants_ahead_x_)
        {
            has_plants_ahead = true;
            break;
        }
    }

    if (!has_plants_ahead)
    {
        for (const auto &point : right_inner->points)
        {
            if (point.x > plants_ahead_x_)
            {
                has_plants_ahead = true;
                break;
            }
        }
    }

    if (!has_plants_ahead)
    {
        RCLCPP_WARN(this->get_logger(), "No plants detected ahead of the robot. Left points: %ld, Right points: %ld",
                    left_inner->size(), right_inner->size());
        // 发布空路径以清空显示
        publish_empty_path(base_header);
        return;
    }

    // fit lines for both inner rows
    auto [left_slope, left_intercept] = this->fit_line(left_inner);
    auto [right_slope, right_intercept] = this->fit_line(right_inner);

    std::pair<float, float> stable_left_line;
    std::pair<float, float> stable_right_line;
    float stable_row_yaw = row_yaw;
    if (!this->update_tracked_lines(
            {left_slope, left_intercept},
            {right_slope, right_intercept},
            row_yaw,
            stable_left_line,
            stable_right_line,
            stable_row_yaw))
    {
        RCLCPP_WARN(this->get_logger(), "No stable row model available; skip this frame");
        publish_empty_path(base_header);
        return;
    }

    const float row_separation = std::abs(stable_left_line.second - stable_right_line.second);

    nav_msgs::msg::Path center_line_path = this->create_corridor_path(
        left_inner,
        right_inner,
        stable_left_line,
        stable_right_line,
        stable_row_yaw,
        base_header);

    // 平滑处理
    nav_msgs::msg::Path smoothed_path = this->smooth_path(center_line_path);

    // Debug point clouds stay in base_frame_. RViz should transform them through TF;
    // do not relabel local points as odom data.
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*projection_cloud, output_cloud);

    sensor_msgs::msg::PointCloud2 left_output;
    PointCloudXYZPtr left_inner_base = this->rotate_cloud_to_base_frame(left_inner, stable_row_yaw);
    pcl::toROSMsg(*left_inner_base, left_output);

    sensor_msgs::msg::PointCloud2 right_output;
    PointCloudXYZPtr right_inner_base = this->rotate_cloud_to_base_frame(right_inner, stable_row_yaw);
    pcl::toROSMsg(*right_inner_base, right_output);

    output_cloud.header = base_header;
    left_output.header = base_header;
    right_output.header = base_header;

    point_cloud_pub_->publish(output_cloud);
    left_row_pub_->publish(left_output);
    right_row_pub_->publish(right_output);
    center_line_pub_->publish(smoothed_path);
    const float association_x = std::clamp(path_length_ * 0.5f, 0.3f, path_length_);
    const float center_offset = 0.5f * ((stable_left_line.first * association_x + stable_left_line.second) +
                                        (stable_right_line.first * association_x + stable_right_line.second));
    publish_detection_diagnostics(
        true,
        static_cast<int>(left_inner->size()),
        static_cast<int>(right_inner->size()),
        stable_row_yaw,
        center_offset,
        stable_left_line,
        stable_right_line,
        static_cast<int>(smoothed_path.poses.size()));

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Published center line. Left points: %ld, Right points: %ld, Row separation: %.2f",
                          left_inner->size(), right_inner->size(), row_separation);
}

void CornRowDetectorProjection::navigation_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!has_navigation_mode_ || msg->data != last_navigation_mode_)
    {
        if (msg->data == "U_TURN" || msg->data == "NEXT_ROW_REACQUIRE")
        {
            reset_tracking_state("navigation mode changed to " + msg->data);
        }
        last_navigation_mode_ = msg->data;
        has_navigation_mode_ = true;
    }
}

// this function will filter and downsample the point cloud
PointCloudXYZPtr CornRowDetectorProjection::preprocess_point_cloud(PointCloudXYZPtr input_cloud)
{
    // Check if input cloud is empty
    if (input_cloud->empty())
    {
        PointCloudXYZPtr empty_cloud(new PointCloudXYZ());
        return empty_cloud;
    }

    // create a new point cloud
    PointCloudXYZPtr filter_cloud(new PointCloudXYZ());
    PointCloudXYZPtr voxelgrid_cloud(new PointCloudXYZ());

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(input_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min_, z_max_);
    pass_z.filter(*filter_cloud);

    // y filter
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(filter_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_min_, y_max_);
    pass_y.filter(*filter_cloud);

    // x filter
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(filter_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_min_, x_max_);
    pass_x.filter(*filter_cloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(filter_cloud);
    voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_grid.filter(*voxelgrid_cloud);

    return voxelgrid_cloud;
}

PointCloudXYZPtr CornRowDetectorProjection::transform_cloud_to_base_frame(PointCloudXYZPtr input_cloud, const std_msgs::msg::Header &header)
{
    PointCloudXYZPtr transformed_cloud(new PointCloudXYZ());

    if (input_cloud->empty())
    {
        return transformed_cloud;
    }

    if (header.frame_id.empty() || header.frame_id == base_frame_)
    {
        *transformed_cloud = *input_cloud;
        transformed_cloud->header = input_cloud->header;
        return transformed_cloud;
    }

    try
    {
        geometry_msgs::msg::TransformStamped transform_msg =
            tf_buffer_->lookupTransform(base_frame_, header.frame_id, tf2::TimePointZero);

        tf2::Transform transform;
        tf2::fromMsg(transform_msg.transform, transform);

        transformed_cloud->points.reserve(input_cloud->points.size());
        for (const auto &point : input_cloud->points)
        {
            tf2::Vector3 point_in(point.x, point.y, point.z);
            tf2::Vector3 point_out = transform * point_in;
            transformed_cloud->points.emplace_back(point_out.x(), point_out.y(), point_out.z());
        }

        transformed_cloud->width = transformed_cloud->points.size();
        transformed_cloud->height = 1;
        transformed_cloud->is_dense = input_cloud->is_dense;
        return transformed_cloud;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Cannot transform point cloud from %s to %s: %s. Using raw cloud as fallback.",
                             header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        *transformed_cloud = *input_cloud;
        return transformed_cloud;
    }
}

PointCloudXYZPtr CornRowDetectorProjection::projection_point_cloud(PointCloudXYZPtr input_cloud)
{
    // Check if input cloud is empty
    if (input_cloud->empty())
    {
        PointCloudXYZPtr empty_cloud(new PointCloudXYZ());
        return empty_cloud;
    }

    // create a new point cloud, which will store the projected points
    pcl::PointCloud<pcl::PointXY>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXY>);

    for (const auto &point : input_cloud->points)
    {
        // only keep x and y, project to xy plane
        pcl::PointXY p;
        p.x = point.x;
        p.y = point.y;
        projected_cloud->points.push_back(p);
    }

    projected_cloud->width = projected_cloud->points.size();
    projected_cloud->height = 1;
    projected_cloud->is_dense = true;

    // Convert PointXY back to PointXYZ with a small offset in Z to make it visible in RVIZ
    PointCloudXYZPtr final_cloud(new PointCloudXYZ());
    final_cloud->header = input_cloud->header;
    final_cloud->points.resize(projected_cloud->size());

    for (size_t i = 0; i < projected_cloud->size(); ++i)
    {
        pcl::PointXYZ point;
        point.x = projected_cloud->points[i].x;
        point.y = projected_cloud->points[i].y;
        point.z = 0.01; // Small offset in Z to make points visible in RVIZ
        final_cloud->points[i] = point;
    }

    final_cloud->width = projected_cloud->width;
    final_cloud->height = 1;
    final_cloud->is_dense = true;

    return final_cloud;
}

float CornRowDetectorProjection::normalize_angle(float angle) const
{
    while (angle > static_cast<float>(M_PI))
    {
        angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle < -static_cast<float>(M_PI))
    {
        angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
}

float CornRowDetectorProjection::normalize_axis_angle(float angle) const
{
    angle = normalize_angle(angle);
    const float half_pi = 0.5f * static_cast<float>(M_PI);
    while (angle > half_pi)
    {
        angle -= static_cast<float>(M_PI);
    }
    while (angle < -half_pi)
    {
        angle += static_cast<float>(M_PI);
    }
    return angle;
}

void CornRowDetectorProjection::reset_tracking_state(const std::string &reason)
{
    has_tracked_lines_ = false;
    has_tracked_row_yaw_ = false;
    has_tracked_global_row_yaw_ = false;
    line_lost_count_ = 0;
    headland_candidate_count_ = 0;
    path_history_.clear();
    RCLCPP_WARN(this->get_logger(), "Reset centerline tracking state: %s", reason.c_str());
}

bool CornRowDetectorProjection::lookup_output_from_base_transform(
    geometry_msgs::msg::TransformStamped &output_from_base)
{
    if (output_frame_ == base_frame_)
    {
        return false;
    }

    try
    {
        output_from_base = tf_buffer_->lookupTransform(output_frame_, base_frame_, tf2::TimePointZero);
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Cannot lookup %s <- %s for global row yaw: %s",
                             output_frame_.c_str(), base_frame_.c_str(), ex.what());
        return false;
    }
}

float CornRowDetectorProjection::estimate_global_row_yaw(
    PointCloudXYZPtr cloud_base,
    const geometry_msgs::msg::TransformStamped &output_from_base)
{
    PointCloudXYZPtr relative_output_cloud(new PointCloudXYZ());
    if (cloud_base->empty())
    {
        return 0.0f;
    }

    tf2::Transform transform;
    tf2::fromMsg(output_from_base.transform, transform);
    const tf2::Vector3 robot_origin(
        output_from_base.transform.translation.x,
        output_from_base.transform.translation.y,
        output_from_base.transform.translation.z);

    relative_output_cloud->points.reserve(cloud_base->points.size());
    for (const auto &point : cloud_base->points)
    {
        const tf2::Vector3 point_base(point.x, point.y, point.z);
        const tf2::Vector3 point_output = transform * point_base;
        pcl::PointXYZ relative_point;
        relative_point.x = point_output.x() - robot_origin.x();
        relative_point.y = point_output.y() - robot_origin.y();
        relative_point.z = point.z;
        relative_output_cloud->points.push_back(relative_point);
    }

    relative_output_cloud->width = relative_output_cloud->points.size();
    relative_output_cloud->height = 1;
    relative_output_cloud->is_dense = cloud_base->is_dense;
    return this->normalize_axis_angle(this->estimate_row_yaw(relative_output_cloud));
}

float CornRowDetectorProjection::estimate_row_yaw(PointCloudXYZPtr cloud)
{
    if (cloud->points.size() < static_cast<size_t>(min_cluster_size_ * 2))
    {
        return 0.0f;
    }

    std::vector<pcl::PointXYZ> roi_points;
    roi_points.reserve(cloud->points.size());
    for (const auto &point : cloud->points)
    {
        if (point.x < x_min_ || point.x > x_max_ || point.y < y_min_ || point.y > y_max_)
        {
            continue;
        }
        roi_points.push_back(point);
    }

    if (roi_points.size() < static_cast<size_t>(min_cluster_size_ * 2))
    {
        return 0.0f;
    }

    auto score_yaw = [this, &roi_points](float yaw)
    {
        const float c = std::cos(yaw);
        const float s = std::sin(yaw);
        std::vector<float> lateral_values;
        lateral_values.reserve(roi_points.size());

        for (const auto &point : roi_points)
        {
            const float row_x = c * point.x + s * point.y;
            const float row_y = -s * point.x + c * point.y;
            if (row_x < x_min_ || row_x > x_max_)
            {
                continue;
            }
            lateral_values.push_back(row_y);
        }

        if (lateral_values.size() < static_cast<size_t>(min_cluster_size_ * 2))
        {
            return -std::numeric_limits<float>::infinity();
        }

        std::sort(lateral_values.begin(), lateral_values.end());

        struct ClusterStats
        {
            int count = 0;
            float sum = 0.0f;
            float sum_sq = 0.0f;
        };

        std::vector<ClusterStats> clusters;
        ClusterStats current;
        current.count = 1;
        current.sum = lateral_values.front();
        current.sum_sq = lateral_values.front() * lateral_values.front();

        for (size_t i = 1; i < lateral_values.size(); ++i)
        {
            const float gap = lateral_values[i] - lateral_values[i - 1];
            if (gap <= lateral_cluster_eps_)
            {
                current.count++;
                current.sum += lateral_values[i];
                current.sum_sq += lateral_values[i] * lateral_values[i];
            }
            else
            {
                clusters.push_back(current);
                current = ClusterStats{};
                current.count = 1;
                current.sum = lateral_values[i];
                current.sum_sq = lateral_values[i] * lateral_values[i];
            }
        }
        clusters.push_back(current);

        float compactness_penalty = 0.0f;
        int valid_points = 0;
        int valid_clusters = 0;
        bool has_left = false;
        bool has_right = false;
        float closest_left_y = std::numeric_limits<float>::max();
        float closest_right_y = -std::numeric_limits<float>::max();

        for (const auto &cluster : clusters)
        {
            if (cluster.count < min_cluster_size_)
            {
                continue;
            }

            const float mean_y = cluster.sum / static_cast<float>(cluster.count);
            const float variance = std::max(0.0f, cluster.sum_sq / static_cast<float>(cluster.count) - mean_y * mean_y);
            compactness_penalty += variance * static_cast<float>(cluster.count);
            valid_points += cluster.count;
            valid_clusters++;

            if (mean_y > 0.0f)
            {
                has_left = true;
                closest_left_y = std::min(closest_left_y, mean_y);
            }
            else if (mean_y < 0.0f)
            {
                has_right = true;
                closest_right_y = std::max(closest_right_y, mean_y);
            }
        }

        if (!has_left || !has_right || valid_clusters < 2)
        {
            return -std::numeric_limits<float>::infinity();
        }

        const float inner_width = closest_left_y - closest_right_y;
        const float width_error = std::abs(inner_width - desired_row_separation_);
        const float width_score = 1.0f - std::clamp(width_error / std::max(desired_row_separation_, 1e-3f), 0.0f, 1.0f);
        const float point_score = static_cast<float>(valid_points);
        const float compactness_score = compactness_penalty / static_cast<float>(std::max(1, valid_points));

        return point_score + 25.0f * width_score - 500.0f * compactness_score;
    };

    float best_yaw = 0.0f;
    float best_score = -std::numeric_limits<float>::infinity();
    const float yaw_step = 0.02f;
    for (float yaw = -max_row_alignment_yaw_; yaw <= max_row_alignment_yaw_ + 1e-5f; yaw += yaw_step)
    {
        const float score = score_yaw(yaw);
        if (score > best_score)
        {
            best_score = score;
            best_yaw = yaw;
        }
    }

    if (!std::isfinite(best_score))
    {
        return 0.0f;
    }

    best_yaw = this->normalize_axis_angle(best_yaw);
    RCLCPP_DEBUG(this->get_logger(), "Estimated row yaw in base frame: %.3f rad, score=%.2f", best_yaw, best_score);
    return best_yaw;
}

float CornRowDetectorProjection::filter_global_row_yaw(float measured_global_row_yaw)
{
    measured_global_row_yaw = this->normalize_axis_angle(measured_global_row_yaw);
    if (!enable_temporal_tracking_)
    {
        tracked_global_row_yaw_ = measured_global_row_yaw;
        has_tracked_global_row_yaw_ = true;
        return measured_global_row_yaw;
    }
    if (!has_tracked_global_row_yaw_)
    {
        tracked_global_row_yaw_ = measured_global_row_yaw;
        has_tracked_global_row_yaw_ = true;
        return tracked_global_row_yaw_;
    }

    const float yaw_delta = this->normalize_axis_angle(measured_global_row_yaw - tracked_global_row_yaw_);
    if (std::abs(yaw_delta) > max_row_yaw_jump_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Reject global row yaw jump: measured=%.3f tracked=%.3f delta=%.3f",
                             measured_global_row_yaw, tracked_global_row_yaw_, yaw_delta);
        return tracked_global_row_yaw_;
    }

    tracked_global_row_yaw_ = this->normalize_axis_angle(
        tracked_global_row_yaw_ + line_filter_alpha_ * yaw_delta);
    return tracked_global_row_yaw_;
}

PointCloudXYZPtr CornRowDetectorProjection::rotate_cloud_to_row_frame(PointCloudXYZPtr cloud, float row_yaw)
{
    PointCloudXYZPtr rotated_cloud(new PointCloudXYZ());
    rotated_cloud->points.reserve(cloud->points.size());

    const float c = std::cos(row_yaw);
    const float s = std::sin(row_yaw);
    for (const auto &point : cloud->points)
    {
        pcl::PointXYZ rotated_point;
        rotated_point.x = c * point.x + s * point.y;
        rotated_point.y = -s * point.x + c * point.y;
        rotated_point.z = point.z;
        rotated_cloud->points.push_back(rotated_point);
    }

    rotated_cloud->width = rotated_cloud->points.size();
    rotated_cloud->height = 1;
    rotated_cloud->is_dense = cloud->is_dense;
    return rotated_cloud;
}

PointCloudXYZPtr CornRowDetectorProjection::rotate_cloud_to_base_frame(PointCloudXYZPtr cloud, float row_yaw)
{
    PointCloudXYZPtr rotated_cloud(new PointCloudXYZ());
    rotated_cloud->points.reserve(cloud->points.size());

    const float c = std::cos(row_yaw);
    const float s = std::sin(row_yaw);
    for (const auto &point : cloud->points)
    {
        pcl::PointXYZ rotated_point;
        rotated_point.x = c * point.x - s * point.y;
        rotated_point.y = s * point.x + c * point.y;
        rotated_point.z = point.z;
        rotated_cloud->points.push_back(rotated_point);
    }

    rotated_cloud->width = rotated_cloud->points.size();
    rotated_cloud->height = 1;
    rotated_cloud->is_dense = cloud->is_dense;
    return rotated_cloud;
}

std::pair<PointCloudXYZPtr, PointCloudXYZPtr> CornRowDetectorProjection::split_left_right_rows(PointCloudXYZPtr input_cloud)
{
    PointCloudXYZPtr left_cloud(new PointCloudXYZ());
    PointCloudXYZPtr right_cloud(new PointCloudXYZ());

    // Check if input cloud is empty
    if (input_cloud->empty())
    {
        return std::make_pair(left_cloud, right_cloud);
    }

    for (const auto &point : input_cloud->points)
    {
        // Split points based on y coordinate (left row: y > 0, right row: y < 0)
        if (point.y > 0)
        {
            left_cloud->points.push_back(point);
        }
        else
        {
            right_cloud->points.push_back(point);
        }
    }

    left_cloud->width = left_cloud->points.size();
    left_cloud->height = 1;
    left_cloud->is_dense = true;

    right_cloud->width = right_cloud->points.size();
    right_cloud->height = 1;
    right_cloud->is_dense = true;

    return std::make_pair(left_cloud, right_cloud);
}

PointCloudXYZPtr CornRowDetectorProjection::extract_innermost_row(PointCloudXYZPtr cloud, bool left)
{
    PointCloudXYZPtr result(new PointCloudXYZ());
    if (cloud->empty())
    {
        return result;
    }

    // Debug: log invocation details
    RCLCPP_DEBUG(this->get_logger(), "extract_innermost_row called: side=%s, cloud_size=%zu, lateral_cluster_eps=%.3f, min_cluster_size=%d",
                 left ? "left" : "right", cloud->size(), lateral_cluster_eps_, min_cluster_size_);

    if (use_simple_inner_row_mode_)
    {
        std::vector<std::pair<float, size_t>> y_idx;
        y_idx.reserve(cloud->points.size());
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            const float y = cloud->points[i].y;
            if ((left && y > 0.0f) || (!left && y < 0.0f))
            {
                y_idx.emplace_back(y, i);
            }
        }

        if (y_idx.empty())
        {
            return result;
        }

        std::sort(y_idx.begin(), y_idx.end(), [](const auto &a, const auto &b)
                  { return a.first < b.first; });

        std::vector<std::vector<size_t>> clusters;
        clusters.emplace_back();
        clusters.back().push_back(y_idx[0].second);
        for (size_t i = 1; i < y_idx.size(); ++i)
        {
            const float gap = y_idx[i].first - y_idx[i - 1].first;
            if (std::abs(gap) <= lateral_cluster_eps_)
            {
                clusters.back().push_back(y_idx[i].second);
            }
            else
            {
                clusters.emplace_back();
                clusters.back().push_back(y_idx[i].second);
            }
        }

        int best_cluster_idx = -1;
        float best_abs_y = std::numeric_limits<float>::max();
        for (size_t ci = 0; ci < clusters.size(); ++ci)
        {
            if (clusters[ci].size() < static_cast<size_t>(min_cluster_size_))
            {
                continue;
            }

            float sum_y = 0.0f;
            for (auto idx : clusters[ci])
            {
                sum_y += cloud->points[idx].y;
            }
            const float mean_y = sum_y / static_cast<float>(clusters[ci].size());
            const float abs_y = std::abs(mean_y);

            if (abs_y < best_abs_y)
            {
                best_abs_y = abs_y;
                best_cluster_idx = static_cast<int>(ci);
            }
        }

        if (best_cluster_idx < 0)
        {
            return result;
        }

        for (auto idx : clusters[best_cluster_idx])
        {
            result->points.push_back(cloud->points[idx]);
        }
        result->width = result->points.size();
        result->height = 1;
        result->is_dense = true;
        RCLCPP_DEBUG(this->get_logger(),
                     "Simple inner row selected: side=%s, clusters=%zu, points=%zu, abs_y=%.3f",
                     left ? "left" : "right", clusters.size(), result->points.size(), best_abs_y);
        return result;
    }

    std::vector<pcl::PointXYZ> innermost_seeds;
    const float seed_x_start = std::max(0.0f, x_min_);
    const float seed_x_end = std::min(path_length_, x_max_);
    const float half_bin_width = innermost_bin_width_ * 0.5f;

    for (float x = seed_x_start; x <= seed_x_end + 1e-4f; x += innermost_bin_width_)
    {
        std::vector<float> lateral_candidates;
        for (const auto &point : cloud->points)
        {
            if (std::abs(point.x - x) > half_bin_width)
            {
                continue;
            }
            if ((left && point.y <= 0.0f) || (!left && point.y >= 0.0f))
            {
                continue;
            }
            lateral_candidates.push_back(point.y);
        }

        if (static_cast<int>(lateral_candidates.size()) < min_section_points_)
        {
            continue;
        }

        std::sort(lateral_candidates.begin(), lateral_candidates.end());
        const float quantile = left ? innermost_quantile_ : (1.0f - innermost_quantile_);
        const size_t selected_idx = static_cast<size_t>(
            std::clamp(std::floor(quantile * static_cast<float>(lateral_candidates.size() - 1)),
                       0.0f,
                       static_cast<float>(lateral_candidates.size() - 1)));

        pcl::PointXYZ seed;
        seed.x = x;
        seed.y = lateral_candidates[selected_idx];
        seed.z = 0.0f;
        innermost_seeds.push_back(seed);
    }

    if (static_cast<int>(innermost_seeds.size()) >= min_innermost_seeds_)
    {
        auto least_squares = [this](const std::vector<pcl::PointXYZ> &points)
        {
            float sum_x = 0.0f;
            float sum_y = 0.0f;
            float sum_xx = 0.0f;
            float sum_xy = 0.0f;
            const size_t n = points.size();

            for (const auto &point : points)
            {
                sum_x += point.x;
                sum_y += point.y;
                sum_xx += point.x * point.x;
                sum_xy += point.x * point.y;
            }

            const float denominator = (n * sum_xx - sum_x * sum_x);
            if (std::abs(denominator) < 1e-6f)
            {
                return std::make_pair(0.0f, sum_y / n);
            }

            float slope = (n * sum_xy - sum_x * sum_y) / denominator;
            float intercept = (sum_y - slope * sum_x) / n;
            if (std::abs(slope) > max_line_slope_)
            {
                slope = (slope > 0.0f) ? max_line_slope_ : -max_line_slope_;
            }
            return std::make_pair(slope, intercept);
        };

        const auto [inner_slope, inner_intercept] = least_squares(innermost_seeds);
        for (const auto &point : cloud->points)
        {
            if ((left && point.y <= 0.0f) || (!left && point.y >= 0.0f))
            {
                continue;
            }

            const float expected_y = inner_slope * point.x + inner_intercept;
            const float residual = std::abs(point.y - expected_y);
            if (residual <= innermost_row_band_width_)
            {
                result->points.push_back(point);
            }
        }

        if (result->size() >= static_cast<size_t>(min_cluster_size_))
        {
            result->width = result->points.size();
            result->height = 1;
            result->is_dense = true;
            RCLCPP_DEBUG(this->get_logger(),
                         "Section-based innermost extraction succeeded: side=%s, seeds=%zu, points=%zu, slope=%.3f, intercept=%.3f",
                         left ? "left" : "right", innermost_seeds.size(), result->size(), inner_slope, inner_intercept);
            return result;
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "Section-based innermost extraction found only %zu points; falling back to lateral clustering.",
                     result->size());
        result->points.clear();
    }

    // Collect (y, index) pairs and sort by y
    std::vector<std::pair<float, size_t>> y_idx;
    y_idx.reserve(cloud->points.size());
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        y_idx.emplace_back(cloud->points[i].y, i);
    }
    std::sort(y_idx.begin(), y_idx.end(), [](const auto &a, const auto &b)
              { return a.first < b.first; });

    // 1D clustering on y coordinate
    std::vector<std::vector<size_t>> clusters;

    if (adaptive_lateral_threshold_ && y_idx.size() > 1)
    {
        // compute gaps between adjacent y
        std::vector<float> gaps;
        gaps.reserve(y_idx.size() - 1);
        for (size_t i = 1; i < y_idx.size(); ++i)
            gaps.push_back(y_idx[i].first - y_idx[i - 1].first);

        // compute median gap
        std::vector<float> gaps_sorted = gaps;
        std::sort(gaps_sorted.begin(), gaps_sorted.end());
        float median_gap = 0.0f;
        if (!gaps_sorted.empty())
        {
            size_t m = gaps_sorted.size();
            if (m % 2 == 1)
                median_gap = gaps_sorted[m / 2];
            else
                median_gap = 0.5f * (gaps_sorted[m / 2 - 1] + gaps_sorted[m / 2]);
        }

        float threshold = std::max(median_gap * gap_multiplier_, lateral_cluster_eps_);
        RCLCPP_DEBUG(this->get_logger(), "Adaptive clustering enabled: median_gap=%.4f, gap_multiplier=%.2f, threshold=%.4f", median_gap, gap_multiplier_, threshold);

        // form clusters by splitting at gaps > threshold
        clusters.emplace_back();
        clusters.back().push_back(y_idx[0].second);
        for (size_t i = 1; i < y_idx.size(); ++i)
        {
            float gap = y_idx[i].first - y_idx[i - 1].first;
            if (gap <= threshold)
            {
                clusters.back().push_back(y_idx[i].second);
            }
            else
            {
                clusters.emplace_back();
                clusters.back().push_back(y_idx[i].second);
                RCLCPP_DEBUG(this->get_logger(), "Split cluster at index %zu (gap=%.4f > threshold=%.4f)", i, gap, threshold);
            }
        }
    }
    else
    {
        clusters.emplace_back();
        clusters.back().push_back(y_idx[0].second);
        for (size_t i = 1; i < y_idx.size(); ++i)
        {
            if (std::abs(y_idx[i].first - y_idx[i - 1].first) <= lateral_cluster_eps_)
            {
                clusters.back().push_back(y_idx[i].second);
            }
            else
            {
                clusters.emplace_back();
                clusters.back().push_back(y_idx[i].second);
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "Fixed-eps clustering used (eps=%.4f)", lateral_cluster_eps_);
    }

    // Debug: log cluster count and per-cluster stats
    RCLCPP_DEBUG(this->get_logger(), "Found %zu clusters on side=%s", clusters.size(), left ? "left" : "right");
    for (size_t ci = 0; ci < clusters.size(); ++ci)
    {
        float sum_y = 0.0f;
        for (auto idx : clusters[ci])
            sum_y += cloud->points[idx].y;
        float mean_y = sum_y / clusters[ci].size();
        RCLCPP_DEBUG(this->get_logger(), "  Cluster %zu: size=%zu, mean_y=%.3f", ci, clusters[ci].size(), mean_y);
    }

    // Choose the cluster that is closest to the robot centerline (innermost)
    int best_cluster_idx = -1;
    float best_metric = 0.0f; // for left: smaller mean_y is better; for right: larger mean_y (less negative) is better

    for (size_t ci = 0; ci < clusters.size(); ++ci)
    {
        float sum_y = 0.0f;
        for (auto idx : clusters[ci])
            sum_y += cloud->points[idx].y;
        float mean_y = sum_y / clusters[ci].size();

        // ignore clusters that are too small
        if (clusters[ci].size() < static_cast<size_t>(min_cluster_size_))
            continue;

        if (left)
        {
            if (mean_y <= 0)
                continue;
            if (best_cluster_idx == -1 || mean_y < best_metric)
            {
                best_metric = mean_y;
                best_cluster_idx = ci;
                RCLCPP_DEBUG(this->get_logger(), "Considered cluster %zu as current best (left): size=%zu, mean_y=%.3f", ci, clusters[ci].size(), mean_y);
            }
        }
        else
        {
            if (mean_y >= 0)
                continue;
            if (best_cluster_idx == -1 || mean_y > best_metric)
            {
                best_metric = mean_y;
                best_cluster_idx = ci;
                RCLCPP_DEBUG(this->get_logger(), "Considered cluster %zu as current best (right): size=%zu, mean_y=%.3f", ci, clusters[ci].size(), mean_y);
            }
        }
    }

    // Fallback: if no cluster meets min size requirement, pick cluster closest to zero with correct sign
    if (best_cluster_idx == -1)
    {
        float best_dist = 1e6;
        for (size_t ci = 0; ci < clusters.size(); ++ci)
        {
            float sum_y = 0.0f;
            for (auto idx : clusters[ci])
                sum_y += cloud->points[idx].y;
            float mean_y = sum_y / clusters[ci].size();

            if (left && mean_y <= 0)
                continue;
            if (!left && mean_y >= 0)
                continue;

            float dist = std::abs(mean_y);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_cluster_idx = ci;
                RCLCPP_DEBUG(this->get_logger(), "Fallback1 selected cluster %zu: size=%zu, mean_y=%.3f", ci, clusters[ci].size(), mean_y);
            }
        }
    }

    // Final fallback: any cluster closest to zero (ignore sign) if still not found
    if (best_cluster_idx == -1)
    {
        float best_dist = 1e6;
        for (size_t ci = 0; ci < clusters.size(); ++ci)
        {
            float sum_y = 0.0f;
            for (auto idx : clusters[ci])
                sum_y += cloud->points[idx].y;
            float mean_y = sum_y / clusters[ci].size();
            float dist = std::abs(mean_y);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_cluster_idx = ci;
                RCLCPP_DEBUG(this->get_logger(), "Final fallback selected cluster %zu: size=%zu, mean_y=%.3f", ci, clusters[ci].size(), mean_y);
            }
        }
    }

    if (best_cluster_idx >= 0)
    {
        for (auto idx : clusters[best_cluster_idx])
            result->points.push_back(cloud->points[idx]);

        // Log final selection
        float sum_y = 0.0f;
        for (auto idx : clusters[best_cluster_idx])
            sum_y += cloud->points[idx].y;
        float mean_y = sum_y / clusters[best_cluster_idx].size();
        RCLCPP_DEBUG(this->get_logger(), "Selected cluster %d: size=%zu, mean_y=%.3f", best_cluster_idx, clusters[best_cluster_idx].size(), mean_y);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No suitable cluster found on side=%s; returning empty result", left ? "left" : "right");
    }

    result->width = result->points.size();
    result->height = 1;
    result->is_dense = true;
    return result;
}

std::pair<float, float> CornRowDetectorProjection::fit_line(PointCloudXYZPtr cloud)
{
    if (cloud->points.size() < 5)
    {
        RCLCPP_DEBUG(this->get_logger(), "Not enough points for line fitting: %ld", cloud->points.size());
        return std::make_pair(0.0f, 0.0f);
    }

    std::vector<pcl::PointXYZ> points;
    points.reserve(cloud->points.size());
    for (const auto &point : cloud->points)
    {
        if (point.x >= line_fit_x_min_)
        {
            points.push_back(point);
        }
    }

    if (points.size() < 5)
    {
        points.assign(cloud->points.begin(), cloud->points.end());
    }

    if (use_parallel_row_model_)
    {
        std::vector<float> lateral_values;
        lateral_values.reserve(points.size());
        for (const auto &point : points)
        {
            lateral_values.push_back(point.y);
        }

        std::sort(lateral_values.begin(), lateral_values.end());
        const size_t mid = lateral_values.size() / 2;
        const float intercept = (lateral_values.size() % 2 == 0)
                                    ? 0.5f * (lateral_values[mid - 1] + lateral_values[mid])
                                    : lateral_values[mid];
        return std::make_pair(0.0f, intercept);
    }

    auto least_squares = [this](const std::vector<pcl::PointXYZ> &points)
    {
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        float sum_xx = 0.0f;
        float sum_xy = 0.0f;
        const size_t n = points.size();

        for (const auto &point : points)
        {
            sum_x += point.x;
            sum_y += point.y;
            sum_xx += point.x * point.x;
            sum_xy += point.x * point.y;
        }

        float denominator = (n * sum_xx - sum_x * sum_x);
        if (std::abs(denominator) < 1e-6)
        {
            return std::make_pair(0.0f, sum_y / n);
        }

        float slope = (n * sum_xy - sum_x * sum_y) / denominator;
        float intercept = (sum_y - slope * sum_x) / n;
        if (std::abs(slope) > max_line_slope_)
        {
            slope = (slope > 0.0f) ? max_line_slope_ : -max_line_slope_;
        }
        return std::make_pair(slope, intercept);
    };

    auto [slope, intercept] = least_squares(points);
    if (!enable_robust_refinement_)
    {
        return std::make_pair(slope, intercept);
    }

    std::vector<pcl::PointXYZ> inliers;
    inliers.reserve(points.size());
    for (const auto &point : points)
    {
        float residual = std::abs(point.y - (slope * point.x + intercept));
        if (residual <= robust_fit_residual_threshold_)
        {
            inliers.push_back(point);
        }
    }

    if (inliers.size() >= 5 && inliers.size() < points.size())
    {
        auto refined = least_squares(inliers);
        RCLCPP_DEBUG(this->get_logger(), "Robust fit kept %zu/%zu points", inliers.size(), points.size());
        return refined;
    }

    return std::make_pair(slope, intercept);
}

float CornRowDetectorProjection::filter_row_yaw(float measured_row_yaw)
{
    measured_row_yaw = this->normalize_axis_angle(measured_row_yaw);
    if (!enable_temporal_tracking_)
    {
        tracked_row_yaw_ = measured_row_yaw;
        has_tracked_row_yaw_ = true;
        return measured_row_yaw;
    }
    if (!has_tracked_row_yaw_)
    {
        tracked_row_yaw_ = measured_row_yaw;
        has_tracked_row_yaw_ = true;
        return tracked_row_yaw_;
    }

    const float yaw_delta = this->normalize_axis_angle(measured_row_yaw - tracked_row_yaw_);
    if (std::abs(yaw_delta) > max_row_yaw_jump_)
    {
        RCLCPP_DEBUG(this->get_logger(), "Reject row yaw jump: measured=%.3f tracked=%.3f", measured_row_yaw, tracked_row_yaw_);
        return tracked_row_yaw_;
    }

    tracked_row_yaw_ = this->normalize_axis_angle(
        tracked_row_yaw_ + line_filter_alpha_ * yaw_delta);
    return tracked_row_yaw_;
}

bool CornRowDetectorProjection::update_tracked_lines(
    const std::pair<float, float> &measured_left_line,
    const std::pair<float, float> &measured_right_line,
    float measured_row_yaw,
    std::pair<float, float> &tracked_left_line,
    std::pair<float, float> &tracked_right_line,
    float &tracked_row_yaw)
{
    const auto line_y = [](const std::pair<float, float> &line, float x)
    {
        return line.first * x + line.second;
    };

    const float association_x = std::clamp(path_length_ * 0.5f, 0.3f, path_length_);
    const float measured_width = std::abs(line_y(measured_left_line, association_x) -
                                          line_y(measured_right_line, association_x));
    bool measurement_valid = true;

    if (std::abs(measured_left_line.first) > max_line_slope_ ||
        std::abs(measured_right_line.first) > max_line_slope_)
    {
        measurement_valid = false;
        RCLCPP_WARN(this->get_logger(), "Reject row measurement due to slope: left=%.2f right=%.2f",
                    measured_left_line.first, measured_right_line.first);
    }

    if (measured_width < min_row_separation_ || measured_width > max_row_separation_)
    {
        measurement_valid = false;
        RCLCPP_WARN(this->get_logger(), "Reject row measurement due to width %.2f", measured_width);
    }

    if (!enable_temporal_tracking_)
    {
        if (!measurement_valid)
        {
            return false;
        }
        tracked_left_line_ = measured_left_line;
        tracked_right_line_ = measured_right_line;
        tracked_row_yaw_ = measured_row_yaw;
        has_tracked_lines_ = true;
        line_lost_count_ = 0;
        tracked_left_line = measured_left_line;
        tracked_right_line = measured_right_line;
        tracked_row_yaw = measured_row_yaw;
        return true;
    }

    if (!has_tracked_lines_)
    {
        if (!measurement_valid)
        {
            return false;
        }

        tracked_left_line_ = measured_left_line;
        tracked_right_line_ = measured_right_line;
        tracked_row_yaw_ = measured_row_yaw;
        has_tracked_lines_ = true;
        line_lost_count_ = 0;
        tracked_left_line = tracked_left_line_;
        tracked_right_line = tracked_right_line_;
        tracked_row_yaw = tracked_row_yaw_;
        return true;
    }

    const float tracked_width = std::abs(line_y(tracked_left_line_, association_x) -
                                         line_y(tracked_right_line_, association_x));
    const float left_jump = std::abs(line_y(measured_left_line, association_x) -
                                     line_y(tracked_left_line_, association_x));
    const float right_jump = std::abs(line_y(measured_right_line, association_x) -
                                      line_y(tracked_right_line_, association_x));
    const float width_jump = std::abs(measured_width - tracked_width);

    if (measurement_valid &&
        (left_jump > max_line_lateral_jump_ ||
         right_jump > max_line_lateral_jump_ ||
         width_jump > max_row_width_jump_))
    {
        measurement_valid = false;
        RCLCPP_WARN(this->get_logger(),
                    "Reject row jump: left=%.2f right=%.2f width=%.2f",
                    left_jump, right_jump, width_jump);
    }

    if (!measurement_valid)
    {
        line_lost_count_++;
        if (line_lost_count_ <= max_line_lost_frames_)
        {
            tracked_left_line = tracked_left_line_;
            tracked_right_line = tracked_right_line_;
            tracked_row_yaw = tracked_row_yaw_;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Use previous row model during unstable detection (%d/%d)",
                                 line_lost_count_, max_line_lost_frames_);
            return true;
        }

        has_tracked_lines_ = false;
        return false;
    }

    tracked_left_line_.first = line_filter_alpha_ * measured_left_line.first +
                               (1.0f - line_filter_alpha_) * tracked_left_line_.first;
    tracked_left_line_.second = line_filter_alpha_ * measured_left_line.second +
                                (1.0f - line_filter_alpha_) * tracked_left_line_.second;
    tracked_right_line_.first = line_filter_alpha_ * measured_right_line.first +
                                (1.0f - line_filter_alpha_) * tracked_right_line_.first;
    tracked_right_line_.second = line_filter_alpha_ * measured_right_line.second +
                                 (1.0f - line_filter_alpha_) * tracked_right_line_.second;
    tracked_row_yaw_ = measured_row_yaw;
    line_lost_count_ = 0;

    tracked_left_line = tracked_left_line_;
    tracked_right_line = tracked_right_line_;
    tracked_row_yaw = tracked_row_yaw_;
    return true;
}

bool CornRowDetectorProjection::estimate_lateral_median(
    PointCloudXYZPtr cloud,
    float x,
    float fallback_y,
    float &estimated_y,
    int &support_count)
{
    std::vector<float> lateral_values;
    lateral_values.reserve(cloud->points.size());
    const float half_width = section_width_ * 0.5f;

    for (const auto &point : cloud->points)
    {
        if (std::abs(point.x - x) <= half_width)
        {
            lateral_values.push_back(point.y);
        }
    }

    support_count = static_cast<int>(lateral_values.size());
    if (support_count < min_section_points_)
    {
        estimated_y = fallback_y;
        return false;
    }

    std::sort(lateral_values.begin(), lateral_values.end());
    const size_t mid = lateral_values.size() / 2;
    if (lateral_values.size() % 2 == 0)
    {
        estimated_y = 0.5f * (lateral_values[mid - 1] + lateral_values[mid]);
    }
    else
    {
        estimated_y = lateral_values[mid];
    }

    return true;
}

nav_msgs::msg::Path CornRowDetectorProjection::transform_path_to_output_frame(const nav_msgs::msg::Path &path_base_link)
{
    if (output_frame_ == base_frame_)
    {
        return path_base_link;
    }

    nav_msgs::msg::Path path_output;
    path_output.header = path_base_link.header;
    path_output.header.frame_id = output_frame_;

    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            output_frame_, base_frame_, tf2::TimePointZero);
        for (const auto &pose_base_link : path_base_link.poses)
        {
            geometry_msgs::msg::PoseStamped pose_output;
            tf2::doTransform(pose_base_link, pose_output, transform);
            pose_output.header.frame_id = output_frame_;
            pose_output.header.stamp = path_base_link.header.stamp;
            path_output.poses.push_back(pose_output);
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Path TF transform failed from %s to %s: %s. Publishing base-frame path.",
                             base_frame_.c_str(), output_frame_.c_str(), ex.what());
        return path_base_link;
    }

    return path_output;
}

void CornRowDetectorProjection::publish_corridor_metrics(
    float width,
    float safety_margin,
    float error_budget,
    float confidence)
{
    last_corridor_width_ = width;
    last_corridor_safety_margin_ = safety_margin;
    last_corridor_error_budget_ = error_budget;
    last_corridor_confidence_ = confidence;

    std_msgs::msg::Float32 width_msg;
    width_msg.data = width;
    corridor_width_pub_->publish(width_msg);

    std_msgs::msg::Float32 safety_msg;
    safety_msg.data = safety_margin;
    corridor_safety_margin_pub_->publish(safety_msg);

    std_msgs::msg::Float32 error_budget_msg;
    error_budget_msg.data = error_budget;
    corridor_error_budget_pub_->publish(error_budget_msg);

    std_msgs::msg::Float32 confidence_msg;
    confidence_msg.data = confidence;
    corridor_confidence_pub_->publish(confidence_msg);
}

void CornRowDetectorProjection::publish_detection_diagnostics(
    bool valid,
    int left_points,
    int right_points,
    float row_yaw,
    float center_offset,
    const std::pair<float, float> &left_line,
    const std::pair<float, float> &right_line,
    int path_points)
{
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label =
        "valid,left_points,right_points,corridor_width_m,safety_margin_m,"
        "published_confidence,row_yaw_rad,center_offset_m,left_slope,left_intercept,"
        "right_slope,right_intercept,line_lost_count,path_points,support_score,"
        "observation_score,width_score,residual_score,safety_score,observed_valid_ratio,"
        "raw_confidence,error_budget_m,support_weight,observation_weight,width_weight,"
        "residual_weight,safety_weight,innermost_enabled,robust_enabled,"
        "temporal_enabled,parallel_enabled";
    msg.data = {
        valid ? 1.0f : 0.0f,
        static_cast<float>(left_points),
        static_cast<float>(right_points),
        last_corridor_width_,
        last_corridor_safety_margin_,
        last_corridor_confidence_,
        row_yaw,
        center_offset,
        left_line.first,
        left_line.second,
        right_line.first,
        right_line.second,
        static_cast<float>(line_lost_count_),
        static_cast<float>(path_points),
        last_quality_support_score_,
        last_quality_observation_score_,
        last_quality_width_score_,
        last_quality_residual_score_,
        last_quality_safety_score_,
        last_quality_observed_valid_ratio_,
        last_raw_corridor_confidence_,
        last_corridor_error_budget_,
        quality_support_weight_,
        quality_observation_weight_,
        quality_width_weight_,
        quality_residual_weight_,
        quality_safety_weight_,
        enable_innermost_row_extraction_ ? 1.0f : 0.0f,
        enable_robust_refinement_ ? 1.0f : 0.0f,
        enable_temporal_tracking_ ? 1.0f : 0.0f,
        use_parallel_row_model_ ? 1.0f : 0.0f};
    msg.layout.dim[0].size = msg.data.size();
    msg.layout.dim[0].stride = msg.data.size();
    detection_diagnostics_pub_->publish(msg);
    publish_headland_detection(valid, left_points, right_points, path_points);
}

void CornRowDetectorProjection::publish_headland_detection(
    bool valid,
    int left_points,
    int right_points,
    int path_points)
{
    const bool invalid_or_short_path =
        !valid ||
        path_points < headland_min_path_points_;
    const bool weak_side_support =
        std::min(left_points, right_points) < headland_min_side_points_;
    const bool low_confidence =
        last_corridor_confidence_ < headland_low_confidence_threshold_;
    const bool candidate =
        enable_headland_detection_ &&
        (invalid_or_short_path || weak_side_support || low_confidence);

    if (candidate)
    {
        headland_candidate_count_++;
    }
    else
    {
        headland_candidate_count_ = 0;
    }

    std_msgs::msg::Bool msg;
    msg.data = enable_headland_detection_ &&
               headland_candidate_count_ >= headland_candidate_frames_;
    headland_detected_pub_->publish(msg);
}

nav_msgs::msg::Path CornRowDetectorProjection::create_corridor_path(
    PointCloudXYZPtr left_cloud,
    PointCloudXYZPtr right_cloud,
    const std::pair<float, float> &left_line,
    const std::pair<float, float> &right_line,
    float row_yaw,
    const std_msgs::msg::Header &header)
{
    nav_msgs::msg::Path center_base;
    nav_msgs::msg::Path left_boundary_base;
    nav_msgs::msg::Path right_boundary_base;
    center_base.header = header;
    left_boundary_base.header = header;
    right_boundary_base.header = header;
    center_base.header.frame_id = base_frame_;
    left_boundary_base.header.frame_id = base_frame_;
    right_boundary_base.header.frame_id = base_frame_;

    float width_sum = 0.0f;
    float support_score_sum = 0.0f;
    float observation_score_sum = 0.0f;
    float width_score_sum = 0.0f;
    float residual_score_sum = 0.0f;
    float safety_score_sum = 0.0f;
    float min_safety_margin = std::numeric_limits<float>::max();
    int observed_valid_sections = 0;
    int total_sections = 0;

    for (float x = 0.0f; x <= path_length_ + 1e-4f; x += path_step_)
    {
        total_sections++;

        const float fallback_left_y = left_line.first * x + left_line.second;
        const float fallback_right_y = right_line.first * x + right_line.second;

        float left_y = fallback_left_y;
        float right_y = fallback_right_y;
        int left_support = 0;
        int right_support = 0;
        const bool left_observed = estimate_lateral_median(left_cloud, x, fallback_left_y, left_y, left_support);
        const bool right_observed = estimate_lateral_median(right_cloud, x, fallback_right_y, right_y, right_support);

        float width = std::abs(left_y - right_y);
        const bool valid_width = width >= min_row_separation_ && width <= max_row_separation_;
        if (!valid_width)
        {
            RCLCPP_DEBUG(this->get_logger(), "Use fitted corridor section x=%.2f because observed width %.2f is invalid", x, width);
            left_y = fallback_left_y;
            right_y = fallback_right_y;
            width = std::abs(left_y - right_y);
        }

        const float fitted_center_y = 0.5f * (fallback_left_y + fallback_right_y);
        const float observed_center_y = 0.5f * (left_y + right_y);
        const float center_y = (1.0f - segmented_center_weight_) * fitted_center_y +
                               segmented_center_weight_ * observed_center_y;
        const float boundary_left_y = (1.0f - segmented_boundary_weight_) * fallback_left_y +
                                      segmented_boundary_weight_ * left_y;
        const float boundary_right_y = (1.0f - segmented_boundary_weight_) * fallback_right_y +
                                       segmented_boundary_weight_ * right_y;
        const float left_fit_residual = std::abs(left_y - fallback_left_y);
        const float right_fit_residual = std::abs(right_y - fallback_right_y);
        const float residual_score = 1.0f - std::clamp((left_fit_residual + right_fit_residual) /
                                                           (2.0f * robust_fit_residual_threshold_ + 1e-6f),
                                                       0.0f, 1.0f);
        const float support_score = std::clamp(
            static_cast<float>(left_support + right_support) / static_cast<float>(2 * std::max(1, min_section_points_)),
            0.0f, 1.0f);
        const bool observed_valid = left_observed && right_observed && valid_width;
        const float observation_score =
            observed_valid ? 1.0f : quality_observation_fallback_score_;
        const float width_score = 1.0f - std::clamp(std::abs(width - desired_row_separation_) /
                                                        std::max(desired_row_separation_, 1e-3f),
                                                    0.0f, 1.0f);
        const float section_error_budget =
            0.5f * (width - platform_width_) - plant_safety_clearance_;
        const float safety_score = std::clamp(
            section_error_budget / std::max(plant_safety_clearance_, 1e-3f),
            0.0f, 1.0f);

        const float corridor_yaw = row_yaw + std::atan2((left_line.first + right_line.first) * 0.5f, 1.0f);
        tf2::Quaternion q;
        q.setRPY(0, 0, corridor_yaw);
        const auto orientation = tf2::toMsg(q);

        const float c = std::cos(row_yaw);
        const float s = std::sin(row_yaw);
        const float center_base_x = c * x - s * center_y;
        const float center_base_y = s * x + c * center_y;
        const float left_base_x = c * x - s * boundary_left_y;
        const float left_base_y = s * x + c * boundary_left_y;
        const float right_base_x = c * x - s * boundary_right_y;
        const float right_base_y = s * x + c * boundary_right_y;

        geometry_msgs::msg::PoseStamped center_pose;
        center_pose.header = center_base.header;
        center_pose.pose.position.x = center_base_x;
        center_pose.pose.position.y = center_base_y;
        center_pose.pose.position.z = 0.0;
        center_pose.pose.orientation = orientation;
        center_base.poses.push_back(center_pose);

        geometry_msgs::msg::PoseStamped left_pose = center_pose;
        left_pose.header = left_boundary_base.header;
        left_pose.pose.position.x = left_base_x;
        left_pose.pose.position.y = left_base_y;
        left_boundary_base.poses.push_back(left_pose);

        geometry_msgs::msg::PoseStamped right_pose = center_pose;
        right_pose.header = right_boundary_base.header;
        right_pose.pose.position.x = right_base_x;
        right_pose.pose.position.y = right_base_y;
        right_boundary_base.poses.push_back(right_pose);

        width_sum += width;
        support_score_sum += support_score;
        observation_score_sum += observation_score;
        width_score_sum += width_score;
        residual_score_sum += residual_score;
        safety_score_sum += safety_score;
        min_safety_margin = std::min(min_safety_margin, 0.5f * (width - platform_width_));
        if (observed_valid)
        {
            observed_valid_sections++;
        }
    }

    if (total_sections == 0)
    {
        last_quality_support_score_ = 0.0f;
        last_quality_observation_score_ = 0.0f;
        last_quality_width_score_ = 0.0f;
        last_quality_residual_score_ = 0.0f;
        last_quality_safety_score_ = 0.0f;
        last_quality_observed_valid_ratio_ = 0.0f;
        last_raw_corridor_confidence_ = 0.0f;
        publish_corridor_metrics(0.0f, 0.0f, -plant_safety_clearance_, 0.0f);
        return transform_path_to_output_frame(center_base);
    }

    const float section_count = static_cast<float>(total_sections);
    last_quality_support_score_ = support_score_sum / section_count;
    last_quality_observation_score_ = observation_score_sum / section_count;
    last_quality_width_score_ = width_score_sum / section_count;
    last_quality_residual_score_ = residual_score_sum / section_count;
    last_quality_safety_score_ = safety_score_sum / section_count;
    last_quality_observed_valid_ratio_ =
        static_cast<float>(observed_valid_sections) / section_count;
    last_raw_corridor_confidence_ = std::clamp(
        quality_support_weight_ * last_quality_support_score_ +
            quality_observation_weight_ * last_quality_observation_score_ +
            quality_width_weight_ * last_quality_width_score_ +
            quality_residual_weight_ * last_quality_residual_score_ +
            quality_safety_weight_ * last_quality_safety_score_,
        0.0f, 1.0f);
    const float mean_width = width_sum / section_count;
    const float error_budget = min_safety_margin - plant_safety_clearance_;
    const float published_confidence =
        enable_quality_evaluation_ ? last_raw_corridor_confidence_ : 1.0f;
    publish_corridor_metrics(
        mean_width,
        min_safety_margin,
        error_budget,
        published_confidence);

    center_line_viz_pub_->publish(center_base);
    left_boundary_pub_->publish(transform_path_to_output_frame(left_boundary_base));
    right_boundary_pub_->publish(transform_path_to_output_frame(right_boundary_base));

    return transform_path_to_output_frame(center_base);
}

nav_msgs::msg::Path CornRowDetectorProjection::smooth_path(const nav_msgs::msg::Path &raw_path)
{
    // 1. 异常值剔除
    nav_msgs::msg::Path removed_outliers_path = this->remove_outliers(raw_path);

    // 2. 空间平滑
    nav_msgs::msg::Path spatial_smoothed_path = this->spatial_smoothing(removed_outliers_path);

    // 3. 时间平滑
    nav_msgs::msg::Path temporal_smoothed_path = this->temporal_smoothing(spatial_smoothed_path);

    return temporal_smoothed_path;
}

nav_msgs::msg::Path CornRowDetectorProjection::remove_outliers(const nav_msgs::msg::Path &path)
{
    if (path.poses.size() < 3)
    {
        return path;
    }

    // 计算点间距离的均值和标准差
    std::vector<double> distances;
    for (size_t i = 1; i < path.poses.size(); i++)
    {
        double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
        double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
        distances.push_back(std::sqrt(dx * dx + dy * dy));
    }

    double sum = std::accumulate(distances.begin(), distances.end(), 0.0);
    double mean = sum / distances.size();

    double sq_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / distances.size() - mean * mean);

    // 剔除异常值
    nav_msgs::msg::Path filtered_path;
    filtered_path.header = path.header;

    // 第一个点总是保留
    filtered_path.poses.push_back(path.poses[0]);

    for (size_t i = 1; i < path.poses.size() - 1; i++)
    {
        double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
        double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // 如果距离在正常范围内，则保留该点
        if (std::abs(distance - mean) <= outlier_threshold_ * stdev)
        {
            filtered_path.poses.push_back(path.poses[i]);
        }
    }

    // 最后一个点总是保留
    if (!path.poses.empty())
    {
        filtered_path.poses.push_back(path.poses.back());
    }

    return filtered_path;
}

nav_msgs::msg::Path CornRowDetectorProjection::spatial_smoothing(const nav_msgs::msg::Path &path)
{
    if (path.poses.size() < 3)
    {
        return path;
    }

    nav_msgs::msg::Path smoothed_path;
    smoothed_path.header = path.header;

    // 第一个点保持不变
    smoothed_path.poses.push_back(path.poses[0]);

    // 对中间点进行平滑处理
    for (size_t i = 1; i < path.poses.size() - 1; i++)
    {
        geometry_msgs::msg::PoseStamped smoothed_pose = path.poses[i];

        // 使用前后点进行平滑
        smoothed_pose.pose.position.x = spatial_smooth_weight_ * path.poses[i].pose.position.x +
                                        (1.0 - spatial_smooth_weight_) * (path.poses[i - 1].pose.position.x + path.poses[i + 1].pose.position.x) / 2.0;

        smoothed_pose.pose.position.y = spatial_smooth_weight_ * path.poses[i].pose.position.y +
                                        (1.0 - spatial_smooth_weight_) * (path.poses[i - 1].pose.position.y + path.poses[i + 1].pose.position.y) / 2.0;

        // 重新计算方向
        if (i < path.poses.size() - 1)
        {
            double dx = smoothed_pose.pose.position.x - smoothed_path.poses.back().pose.position.x;
            double dy = smoothed_pose.pose.position.y - smoothed_path.poses.back().pose.position.y;
            double yaw = std::atan2(dy, dx);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            smoothed_pose.pose.orientation = tf2::toMsg(q);
        }

        smoothed_path.poses.push_back(smoothed_pose);
    }

    // 最后一个点保持不变
    smoothed_path.poses.push_back(path.poses.back());

    return smoothed_path;
}

nav_msgs::msg::Path CornRowDetectorProjection::temporal_smoothing(const nav_msgs::msg::Path &path)
{
    if (!enable_temporal_tracking_)
    {
        path_history_.clear();
        return path;
    }

    // 添加到历史记录
    path_history_.push_back(path);

    // 保持历史记录窗口大小
    if (path_history_.size() > static_cast<size_t>(time_window_size_))
    {
        path_history_.pop_front();
    }

    if (path_history_.size() < 2)
    {
        return path;
    }

    nav_msgs::msg::Path smoothed_path;
    smoothed_path.header = path.header;

    // 对每个点进行时间平滑
    for (size_t i = 0; i < path.poses.size(); i++)
    {
        geometry_msgs::msg::PoseStamped smoothed_pose = path.poses[i];
        smoothed_pose.header = path.header;

        double sum_x = 0.0, sum_y = 0.0;
        size_t count = 0;

        // 遍历历史记录中的对应点
        for (const auto &history_path : path_history_)
        {
            if (i < history_path.poses.size())
            {
                sum_x += history_path.poses[i].pose.position.x;
                sum_y += history_path.poses[i].pose.position.y;
                count++;
            }
        }

        if (count > 0)
        {
            smoothed_pose.pose.position.x = sum_x / count;
            smoothed_pose.pose.position.y = sum_y / count;

            // 重新计算方向
            if (i > 0 && smoothed_path.poses.size() > 0)
            {
                double dx = smoothed_pose.pose.position.x - smoothed_path.poses.back().pose.position.x;
                double dy = smoothed_pose.pose.position.y - smoothed_path.poses.back().pose.position.y;
                double yaw = std::atan2(dy, dx);

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                smoothed_pose.pose.orientation = tf2::toMsg(q);
            }
        }

        smoothed_path.poses.push_back(smoothed_pose);
    }

    return smoothed_path;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CornRowDetectorProjection>());
    rclcpp::shutdown();
    return 0;
}

void CornRowDetectorProjection::publish_empty_path(const std_msgs::msg::Header &header)
{
    if (has_tracked_lines_ || has_tracked_row_yaw_ || has_tracked_global_row_yaw_)
    {
        line_lost_count_++;
        if (line_lost_count_ > max_line_lost_frames_)
        {
            reset_tracking_state("empty detection exceeded max_line_lost_frames");
        }
    }

    nav_msgs::msg::Path empty_output_path;
    empty_output_path.header = header;
    empty_output_path.header.frame_id = output_frame_;
    center_line_pub_->publish(empty_output_path);
    left_boundary_pub_->publish(empty_output_path);
    right_boundary_pub_->publish(empty_output_path);

    nav_msgs::msg::Path empty_base_path;
    empty_base_path.header = header;
    empty_base_path.header.frame_id = base_frame_;
    center_line_viz_pub_->publish(empty_base_path);

    sensor_msgs::msg::PointCloud2 empty_cloud;
    empty_cloud.header = header;
    empty_cloud.header.frame_id = base_frame_;
    point_cloud_pub_->publish(empty_cloud);
    left_row_pub_->publish(empty_cloud);
    right_row_pub_->publish(empty_cloud);

    path_history_.clear();
    last_quality_support_score_ = 0.0f;
    last_quality_observation_score_ = 0.0f;
    last_quality_width_score_ = 0.0f;
    last_quality_residual_score_ = 0.0f;
    last_quality_safety_score_ = 0.0f;
    last_quality_observed_valid_ratio_ = 0.0f;
    last_raw_corridor_confidence_ = 0.0f;
    publish_corridor_metrics(0.0f, 0.0f, -plant_safety_clearance_, 0.0f);
    publish_detection_diagnostics(false, 0, 0, 0.0f, 0.0f, {0.0f, 0.0f}, {0.0f, 0.0f}, 0);
}
