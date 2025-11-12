#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"                 // provide pcl point cloud type
#include "pcl/point_types.h"                 // provide pcl point types
#include <pcl_conversions/pcl_conversions.h> // provite ros2 to pcl conversion functions
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <numeric>
#include <tf2/LinearMath/Quaternion.h>
#include "centerline_extraction/corn_row_detector_projection.hpp"

// define point cloud type
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

CornRowDetectorProjection::CornRowDetectorProjection() : Node("corn_row_detector_projection")
{
    // create subscribers and publishers
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10, std::bind(&CornRowDetectorProjection::point_cloud_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_combined", 10, std::bind(&CornRowDetectorProjection::odom_callback, this, std::placeholders::_1));
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_projected", 10);
    left_row_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("left_row_points", 10);
    right_row_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("right_row_points", 10);
    center_line_pub_ = this->create_publisher<nav_msgs::msg::Path>("corn_row_center_line", 10);
    center_line_viz_pub_ = this->create_publisher<nav_msgs::msg::Path>("corn_row_center_line_viz", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // declare parameters
    this->declare_parameter<float>("z_min", 0.0);
    this->declare_parameter<float>("z_max", 0.5);
    this->declare_parameter<float>("voxel_size", 0.02);
    this->declare_parameter<int>("time_window_size", 10);
    this->declare_parameter<float>("spatial_smooth_weight", 0.8);
    this->declare_parameter<float>("outlier_threshold", 0.8);
    this->declare_parameter<float>("centerline_length", 1.5);

    // get parameters
    this->get_parameter("z_min", z_min_);
    this->get_parameter("z_max", z_max_);
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("time_window_size", time_window_size_);
    this->get_parameter("spatial_smooth_weight", spatial_smooth_weight_);
    this->get_parameter("outlier_threshold", outlier_threshold_);
    this->get_parameter("centerline_length", ceneterline_length_);
}

void CornRowDetectorProjection::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_current_x_ = msg->pose.pose.position.x; // 获取小车当前x坐标
    robot_current_y_ = msg->pose.pose.position.y; // （可选）y坐标
}

// callback function
void CornRowDetectorProjection::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // convert pointcloud2 to pcl
    PointCloudXYZPtr cloud(new PointCloudXYZ());
    pcl::fromROSMsg(*msg, *cloud);

    // preprocess point cloudS
    PointCloudXYZPtr preprocessed_cloud = this->preprocess_point_cloud(cloud);

    // projection point cloud
    PointCloudXYZPtr projection_cloud = this->projection_point_cloud(preprocessed_cloud);

    // split point cloud to left and right rows
    auto [left_row_cloud, right_row_cloud] = this->split_left_right_rows(projection_cloud);

    // Check if we have enough points to generate a meaningful center line
    // Only proceed if both sides have sufficient points
    if (left_row_cloud->size() < 10 || right_row_cloud->size() < 10)
    {
        RCLCPP_WARN(this->get_logger(), "Insufficient points for center line generation. Left points: %ld, Right points: %ld",
                    left_row_cloud->size(), right_row_cloud->size());
        // 发布空路径以清空显示
        publish_empty_path(msg->header);
        return;
    }

    // Check if there are plants in front of the robot (x > 0.5)
    bool has_plants_ahead = false;
    for (const auto &point : left_row_cloud->points)
    {
        if (point.x > 0.5)
        { // At least 0.5 meters ahead
            has_plants_ahead = true;
            break;
        }
    }

    if (!has_plants_ahead)
    {
        for (const auto &point : right_row_cloud->points)
        {
            if (point.x > 0.5)
            { // At least 0.5 meters ahead
                has_plants_ahead = true;
                break;
            }
        }
    }

    if (!has_plants_ahead)
    {
        RCLCPP_WARN(this->get_logger(), "No plants detected ahead of the robot. Left points: %ld, Right points: %ld",
                    left_row_cloud->size(), right_row_cloud->size());
        // 发布空路径以清空显示
        publish_empty_path(msg->header);
        return;
    }

    // fit lines for both rows
    auto [left_slope, left_intercept] = this->fit_line(left_row_cloud);
    auto [right_slope, right_intercept] = this->fit_line(right_row_cloud);

    // Check if the lines are reasonable (not too steep)
    if (std::abs(left_slope) > 5.0 || std::abs(right_slope) > 5.0)
    {
        RCLCPP_WARN(this->get_logger(), "Unreasonable line slopes. Left slope: %.2f, Right slope: %.2f",
                    left_slope, right_slope);
        // 发布空路径以清空显示
        publish_empty_path(msg->header);
        return;
    }

    // Check if the rows are properly separated
    float row_separation = std::abs(left_intercept - right_intercept);
    if (row_separation < 0.3 || row_separation > 3.0)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid row separation: %.2f meters", row_separation);
        // 发布空路径以清空显示
        publish_empty_path(msg->header);
        return;
    }

    // create center line path
    nav_msgs::msg::Path center_line_path = this->create_path((left_slope + right_slope) / 2.0,
                                                             (left_intercept + right_intercept) / 2.0,
                                                             msg->header);

    // 平滑处理
    nav_msgs::msg::Path smoothed_path = this->smooth_path(center_line_path);

    // transform point to ROS msg
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*projection_cloud, output_cloud);

    sensor_msgs::msg::PointCloud2 left_output;
    pcl::toROSMsg(*left_row_cloud, left_output);

    sensor_msgs::msg::PointCloud2 right_output;
    pcl::toROSMsg(*right_row_cloud, right_output);

    // publish point clouds
    output_cloud.header = msg->header;
    left_output.header = msg->header;
    right_output.header = msg->header;

    point_cloud_pub_->publish(output_cloud);
    left_row_pub_->publish(left_output);
    right_row_pub_->publish(right_output);
    center_line_pub_->publish(smoothed_path);

    RCLCPP_INFO(this->get_logger(), "Published center line. Left points: %ld, Right points: %ld, Row separation: %.2f",
                left_row_cloud->size(), right_row_cloud->size(), row_separation);
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

    // high pass filter
    // RCLCPP_INFO(this->get_logger(), "High pass filter");
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(input_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min_, z_max_);
    pass_z.filter(*filter_cloud);

    // y filter
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(filter_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-0.5f, 0.5f);
    pass_y.filter(*filter_cloud);

    // voxel grid
    // RCLCPP_INFO(this->get_logger(), "Voxel grid");
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(filter_cloud);
    voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_grid.filter(*voxelgrid_cloud);

    return voxelgrid_cloud;
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

std::pair<float, float> CornRowDetectorProjection::fit_line(PointCloudXYZPtr cloud)
{
    // Return a horizontal line at y=0 if not enough points
    if (cloud->points.size() < 5)
    {
        RCLCPP_DEBUG(this->get_logger(), "Not enough points for line fitting: %ld", cloud->points.size());
        return std::make_pair(0.0f, 0.0f);
    }

    // Calculate line using least squares method
    float sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    size_t n = cloud->points.size();

    for (const auto &point : cloud->points)
    {
        sum_x += point.x;
        sum_y += point.y;
        sum_xx += point.x * point.x;
        sum_xy += point.x * point.y;
    }

    // Calculate slope and intercept
    float denominator = (n * sum_xx - sum_x * sum_x);
    if (std::abs(denominator) < 1e-6)
    {
        // Vertical line case - return horizontal line at average y
        RCLCPP_DEBUG(this->get_logger(), "Vertical line case, returning horizontal line at y=%.2f", sum_y / n);
        return std::make_pair(0.0f, sum_y / n);
    }

    float slope = (n * sum_xy - sum_x * sum_y) / denominator;
    float intercept = (sum_y - slope * sum_x) / n;

    // Check if the line is reasonable
    if (std::abs(slope) > 10.0)
    {
        RCLCPP_DEBUG(this->get_logger(), "Unreasonable slope %.2f, limiting to 10.0", slope);
        slope = (slope > 0) ? 10.0 : -10.0;
    }

    return std::make_pair(slope, intercept);
}

nav_msgs::msg::Path CornRowDetectorProjection::create_path(float slope, float intercept, const std_msgs::msg::Header &header)
{
    nav_msgs::msg::Path path_odom; // 存储odom坐标系下的路径（用于控制逻辑）
    path_odom.header = header;     // 原header（如odom坐标系）

    // 1. 生成odom坐标系下的动态路径（当前x到x+5米，同之前的逻辑）
    float x_start = robot_current_x_;
    float x_end = robot_current_x_ + ceneterline_length_;
    for (float x = x_start; x <= x_end; x += 0.05)
    {
        geometry_msgs::msg::PoseStamped pose_odom;
        pose_odom.header = header;
        pose_odom.pose.position.x = x;
        pose_odom.pose.position.y = slope * x + intercept;
        pose_odom.pose.position.z = 0.0;
        // 计算朝向（略，同之前）
        path_odom.poses.push_back(pose_odom);
    }

    // 2. 将路径从odom坐标系转换到base_link坐标系（用于可视化）
    nav_msgs::msg::Path path_base_link;
    path_base_link.header.frame_id = "base_link"; // 小车本体坐标系
    // path_base_link.header.stamp = this->now();
    path_base_link.header.stamp = path_odom.header.stamp;

    try
    {
        // 获取odom到base_link的变换（小车在odom中的位姿）
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "base_link", "odom_combined", tf2::TimePointZero); // 从odom到base_link

        for (const auto &pose_odom : path_odom.poses)
        {
            geometry_msgs::msg::PoseStamped pose_base_link;
            // 坐标转换：odom下的点 -> base_link下的点
            tf2::doTransform(pose_odom, pose_base_link, transform);
            path_base_link.poses.push_back(pose_base_link);
        }
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF转换失败: %s", ex.what());
        return path_odom; // 转换失败时返回原odom坐标系路径
    }

    // 3. 发布转换后的base_link坐标系路径（用于RViz可视化）
    // 注意：控制逻辑仍需使用odom坐标系的路径，因此需新增一个可视化专用发布者
    // 在类中新增发布者：rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr center_line_viz_pub_;
    center_line_viz_pub_->publish(path_base_link);

    // 返回odom坐标系的路径（供PID控制器使用，不影响控制逻辑）
    return path_odom;
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
    nav_msgs::msg::Path empty_path;
    empty_path.header = header;
    center_line_pub_->publish(empty_path);
}
