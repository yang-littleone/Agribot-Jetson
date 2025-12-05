#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <numeric>
#include <deque>
#include "centerline_extraction/corn_row_detector_projection.hpp"

// 定义点云类型
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// 类成员初始化
CornRowDetectorProjection::CornRowDetectorProjection() : Node("corn_row_detector_projection")
{
    // 创建订阅者和发布者
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

    // 声明参数（增加路径步长和最大斜率限制）
    this->declare_parameter<float>("z_min", 0.0);
    this->declare_parameter<float>("z_max", 0.5);
    this->declare_parameter<float>("voxel_size", 0.02);
    this->declare_parameter<int>("time_window_size", 3);  // 减小时间窗口，提升响应速度
    this->declare_parameter<float>("spatial_smooth_weight", 0.6);  // 降低空间平滑权重，提升跟随性
    this->declare_parameter<float>("outlier_threshold", 0.8);
    this->declare_parameter<float>("centerline_length", 1.5);
    this->declare_parameter<float>("path_step", 0.02);  // 更小的步长，路径更精细
    this->declare_parameter<float>("max_slope", 3.0);   // 限制最大斜率，避免路径偏移

    // 获取参数
    this->get_parameter("z_min", z_min_);
    this->get_parameter("z_max", z_max_);
    this->get_parameter("voxel_size", voxel_size_);
    this->get_parameter("time_window_size", time_window_size_);
    this->get_parameter("spatial_smooth_weight", spatial_smooth_weight_);
    this->get_parameter("outlier_threshold", outlier_threshold_);
    this->get_parameter("centerline_length", ceneterline_length_);
    this->get_parameter("path_step", path_step_);
    this->get_parameter("max_slope", max_slope_);

    // 初始化历史路径队列
    path_history_.clear();
}

void CornRowDetectorProjection::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_current_x_ = msg->pose.pose.position.x;
    robot_current_y_ = msg->pose.pose.position.y;
}

void CornRowDetectorProjection::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 转换ROS点云到PCL
    PointCloudXYZPtr cloud(new PointCloudXYZ());
    pcl::fromROSMsg(*msg, *cloud);

    // 预处理点云
    PointCloudXYZPtr preprocessed_cloud = this->preprocess_point_cloud(cloud);

    // 投影到XY平面
    PointCloudXYZPtr projection_cloud = this->projection_point_cloud(preprocessed_cloud);

    // 分割左右行
    auto [left_row_cloud, right_row_cloud] = this->split_left_right_rows(projection_cloud);

    // 检查点云数量
    if (left_row_cloud->size() < 10 || right_row_cloud->size() < 10)
    {
        RCLCPP_WARN(this->get_logger(), "Insufficient points: Left %ld, Right %ld",
                    left_row_cloud->size(), right_row_cloud->size());
        publish_empty_path(msg->header);
        return;
    }

    // 检查前方是否有作物
    bool has_plants_ahead = false;
    float min_x_ahead = 0.2;  // 降低检测阈值，提升灵敏度
    for (const auto &point : left_row_cloud->points)
    {
        if (point.x > min_x_ahead)
        {
            has_plants_ahead = true;
            break;
        }
    }
    if (!has_plants_ahead)
    {
        for (const auto &point : right_row_cloud->points)
        {
            if (point.x > min_x_ahead)
            {
                has_plants_ahead = true;
                break;
            }
        }
    }
    if (!has_plants_ahead)
    {
        RCLCPP_WARN(this->get_logger(), "No plants ahead (x > %.2f)", min_x_ahead);
        publish_empty_path(msg->header);
        return;
    }

    // 拟合左右行直线
    auto [left_slope, left_intercept] = this->fit_line(left_row_cloud);
    auto [right_slope, right_intercept] = this->fit_line(right_row_cloud);

    // 限制斜率范围
    left_slope = std::clamp(left_slope, -max_slope_, max_slope_);
    right_slope = std::clamp(right_slope, -max_slope_, max_slope_);

    // 检查行间距
    float row_separation = std::abs(left_intercept - right_intercept);
    if (row_separation < 0.3 || row_separation > 3.0)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid row separation: %.2f m", row_separation);
        publish_empty_path(msg->header);
        return;
    }

    // 计算中心线参数（加权平均，结合点云密度）
    float avg_slope = (left_slope + right_slope) / 2.0;
    float avg_intercept = (left_intercept + right_intercept) / 2.0;

    // 生成中心线（仅base_link坐标系，保证和点云同坐标系）
    nav_msgs::msg::Path center_line_path = this->create_path(avg_slope, avg_intercept, msg->header);

    // 轻量化平滑：降低平滑强度，优先响应新点云
    nav_msgs::msg::Path smoothed_path = this->smooth_path(center_line_path);

    // 发布点云和路径
    publish_point_clouds(projection_cloud, left_row_cloud, right_row_cloud, msg->header);
    center_line_pub_->publish(smoothed_path);
    center_line_viz_pub_->publish(smoothed_path);  // 可视化和控制用同一坐标系

    RCLCPP_DEBUG(this->get_logger(), "Published center line: slope=%.2f, intercept=%.2f, separation=%.2f",
                 avg_slope, avg_intercept, row_separation);
}

// 预处理点云（增加x方向滤波，只保留前方有效区域）
PointCloudXYZPtr CornRowDetectorProjection::preprocess_point_cloud(PointCloudXYZPtr input_cloud)
{
    if (input_cloud->empty())
    {
        return PointCloudXYZPtr(new PointCloudXYZ());
    }

    PointCloudXYZPtr filter_cloud(new PointCloudXYZ());
    PointCloudXYZPtr voxelgrid_cloud(new PointCloudXYZ());

    // Z轴滤波
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(input_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_min_, z_max_);
    pass_z.filter(*filter_cloud);

    // Y轴滤波
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(filter_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-0.8f, 0.8f);  // 扩大y范围，避免漏点
    pass_y.filter(*filter_cloud);

    // X轴滤波：只保留小车前方0~2m的点（聚焦有效区域）
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(filter_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(0.0f, 2.0f);
    pass_x.filter(*filter_cloud);

    // 体素下采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(filter_cloud);
    voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_grid.filter(*voxelgrid_cloud);

    return voxelgrid_cloud;
}

// 投影点云到XY平面（无修改）
PointCloudXYZPtr CornRowDetectorProjection::projection_point_cloud(PointCloudXYZPtr input_cloud)
{
    if (input_cloud->empty())
    {
        return PointCloudXYZPtr(new PointCloudXYZ());
    }

    pcl::PointCloud<pcl::PointXY>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXY>);
    for (const auto &point : input_cloud->points)
    {
        pcl::PointXY p;
        p.x = point.x;
        p.y = point.y;
        projected_cloud->points.push_back(p);
    }

    projected_cloud->width = projected_cloud->points.size();
    projected_cloud->height = 1;
    projected_cloud->is_dense = true;

    PointCloudXYZPtr final_cloud(new PointCloudXYZ());
    final_cloud->header = input_cloud->header;
    final_cloud->points.resize(projected_cloud->size());

    for (size_t i = 0; i < projected_cloud->size(); ++i)
    {
        pcl::PointXYZ point;
        point.x = projected_cloud->points[i].x;
        point.y = projected_cloud->points[i].y;
        point.z = 0.01;  // RVIZ可见
        final_cloud->points[i] = point;
    }

    final_cloud->width = projected_cloud->width;
    final_cloud->height = 1;
    final_cloud->is_dense = true;

    return final_cloud;
}

// 分割左右行（优化分割逻辑，基于点云重心而非简单y>0）
std::pair<PointCloudXYZPtr, PointCloudXYZPtr> CornRowDetectorProjection::split_left_right_rows(PointCloudXYZPtr input_cloud)
{
    PointCloudXYZPtr left_cloud(new PointCloudXYZ());
    PointCloudXYZPtr right_cloud(new PointCloudXYZ());

    if (input_cloud->empty())
    {
        return {left_cloud, right_cloud};
    }

    // 计算所有点的y坐标均值，作为分割基准（适配非对称点云）
    float mean_y = 0.0f;
    for (const auto &point : input_cloud->points)
    {
        mean_y += point.y;
    }
    mean_y /= input_cloud->size();

    // 基于均值分割，而非固定y=0
    for (const auto &point : input_cloud->points)
    {
        if (point.y > mean_y)
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

    return {left_cloud, right_cloud};
}

// 直线拟合（增加权重，优先拟合前方点）
std::pair<float, float> CornRowDetectorProjection::fit_line(PointCloudXYZPtr cloud)
{
    if (cloud->points.size() < 5)
    {
        return {0.0f, 0.0f};
    }

    float sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0, sum_w = 0;
    size_t n = cloud->points.size();

    // 加权拟合：x越大（越远）权重越高，提升远处点的影响
    for (const auto &point : cloud->points)
    {
        float weight = std::max(0.5f, point.x);  // 权重随x增大而增加
        sum_x += point.x * weight;
        sum_y += point.y * weight;
        sum_xx += point.x * point.x * weight;
        sum_xy += point.x * point.y * weight;
        sum_w += weight;
    }

    float denominator = (sum_w * sum_xx - sum_x * sum_x);
    if (std::abs(denominator) < 1e-6)
    {
        float avg_y = sum_y / sum_w;
        return {0.0f, avg_y};
    }

    float slope = (sum_w * sum_xy - sum_x * sum_y) / denominator;
    float intercept = (sum_y - slope * sum_x) / sum_w;

    // 限制斜率
    slope = std::clamp(slope, -max_slope_, max_slope_);

    return {slope, intercept};
}

// 生成路径（仅base_link坐标系，适配点云实际分布）
nav_msgs::msg::Path CornRowDetectorProjection::create_path(float slope, float intercept, const std_msgs::msg::Header &header)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "base_link";  // 统一使用base_link，和点云同坐标系
    path.header.stamp = this->get_clock()->now();  // 使用当前时间，避免时间戳错位

    // 动态确定x范围：基于点云的最大x值，而非固定长度
    float x_max = std::min(ceneterline_length_, 2.0f);  // 最大不超过2m
    float x_start = 0.05;  // 从车头前5cm开始

    // 生成路径点，步长更小，更精细
    for (float x = x_start; x <= x_max; x += path_step_)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;

        // 计算y值：严格跟随拟合直线
        pose.pose.position.x = x;
        pose.pose.position.y = slope * x + intercept;
        pose.pose.position.z = 0.0;

        // 计算朝向：基于当前点和下一个点的方向
        float next_x = x + path_step_;
        float next_y = slope * next_x + intercept;
        double yaw = atan2(next_y - pose.pose.position.y, next_x - pose.pose.position.x);
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);

        path.poses.push_back(pose);
    }

    return path;
}

// 平滑路径（轻量化，降低历史依赖）
nav_msgs::msg::Path CornRowDetectorProjection::smooth_path(const nav_msgs::msg::Path &raw_path)
{
    if (raw_path.poses.size() < 3)
    {
        return raw_path;
    }

    // 1. 轻度异常值剔除
    nav_msgs::msg::Path filtered_path = remove_outliers(raw_path);

    // 2. 轻度空间平滑
    nav_msgs::msg::Path spatial_smoothed = spatial_smoothing(filtered_path);

    // 3. 极轻度时间平滑（窗口缩小）
    nav_msgs::msg::Path temporal_smoothed = temporal_smoothing(spatial_smoothed);

    return temporal_smoothed;
}

// 异常值剔除（更宽松的阈值）
nav_msgs::msg::Path CornRowDetectorProjection::remove_outliers(const nav_msgs::msg::Path &path)
{
    if (path.poses.size() < 3)
    {
        return path;
    }

    std::vector<double> distances;
    for (size_t i = 1; i < path.poses.size(); i++)
    {
        double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
        double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
        distances.push_back(std::sqrt(dx*dx + dy*dy));
    }

    double mean = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    double sq_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / distances.size() - mean*mean);

    nav_msgs::msg::Path filtered_path;
    filtered_path.header = path.header;
    filtered_path.poses.push_back(path.poses[0]);

    // 更宽松的阈值：1.5倍标准差
    for (size_t i = 1; i < path.poses.size()-1; i++)
    {
        double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
        double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);

        if (std::abs(distance - mean) <= 1.5 * outlier_threshold_ * stdev)
        {
            filtered_path.poses.push_back(path.poses[i]);
        }
    }
    filtered_path.poses.push_back(path.poses.back());

    return filtered_path;
}

// 空间平滑（降低权重，提升响应）
nav_msgs::msg::Path CornRowDetectorProjection::spatial_smoothing(const nav_msgs::msg::Path &path)
{
    if (path.poses.size() < 3)
    {
        return path;
    }

    nav_msgs::msg::Path smoothed_path;
    smoothed_path.header = path.header;
    smoothed_path.poses.push_back(path.poses[0]);

    // 降低平滑权重：0.6（原始0.8），更多保留当前点
    for (size_t i = 1; i < path.poses.size()-1; i++)
    {
        geometry_msgs::msg::PoseStamped pose = path.poses[i];
        pose.pose.position.x = spatial_smooth_weight_ * pose.pose.position.x +
                              (1 - spatial_smooth_weight_) * (path.poses[i-1].pose.position.x + path.poses[i+1].pose.position.x)/2;
        pose.pose.position.y = spatial_smooth_weight_ * pose.pose.position.y +
                              (1 - spatial_smooth_weight_) * (path.poses[i-1].pose.position.y + path.poses[i+1].pose.position.y)/2;

        // 重新计算朝向
        double dx = pose.pose.position.x - smoothed_path.poses.back().pose.position.x;
        double dy = pose.pose.position.y - smoothed_path.poses.back().pose.position.y;
        double yaw = atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);

        smoothed_path.poses.push_back(pose);
    }
    smoothed_path.poses.push_back(path.poses.back());

    return smoothed_path;
}

// 时间平滑（缩小窗口，降低历史依赖）
nav_msgs::msg::Path CornRowDetectorProjection::temporal_smoothing(const nav_msgs::msg::Path &path)
{
    path_history_.push_back(path);
    // 窗口缩小到3帧（原始10帧），快速响应新数据
    if (path_history_.size() > static_cast<size_t>(std::min(time_window_size_, 3)))
    {
        path_history_.pop_front();
    }

    if (path_history_.size() < 2)
    {
        return path;
    }

    nav_msgs::msg::Path smoothed_path;
    smoothed_path.header = path.header;

    for (size_t i = 0; i < path.poses.size(); i++)
    {
        geometry_msgs::msg::PoseStamped pose = path.poses[i];
        double sum_x = 0.0, sum_y = 0.0;
        size_t count = 0;

        for (const auto &h_path : path_history_)
        {
            if (i < h_path.poses.size())
            {
                sum_x += h_path.poses[i].pose.position.x;
                sum_y += h_path.poses[i].pose.position.y;
                count++;
            }
        }

        if (count > 0)
        {
            // 时间平滑权重：仅平均最近2-3帧
            pose.pose.position.x = sum_x / count;
            pose.pose.position.y = sum_y / count;

            if (i > 0)
            {
                double dx = pose.pose.position.x - smoothed_path.poses.back().pose.position.x;
                double dy = pose.pose.position.y - smoothed_path.poses.back().pose.position.y;
                double yaw = atan2(dy, dx);
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                pose.pose.orientation = tf2::toMsg(q);
            }
        }
        smoothed_path.poses.push_back(pose);
    }

    return smoothed_path;
}

// 辅助函数：发布点云
void CornRowDetectorProjection::publish_point_clouds(
    PointCloudXYZPtr proj_cloud, PointCloudXYZPtr left_cloud, PointCloudXYZPtr right_cloud, 
    const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::PointCloud2 output, left_output, right_output;
    pcl::toROSMsg(*proj_cloud, output);
    pcl::toROSMsg(*left_cloud, left_output);
    pcl::toROSMsg(*right_cloud, right_output);

    output.header = header;
    left_output.header = header;
    right_output.header = header;

    point_cloud_pub_->publish(output);
    left_row_pub_->publish(left_output);
    right_row_pub_->publish(right_output);
}

// 发布空路径
void CornRowDetectorProjection::publish_empty_path(const std_msgs::msg::Header &header)
{
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = "base_link";
    empty_path.header.stamp = this->get_clock()->now();
    center_line_pub_->publish(empty_path);
    center_line_viz_pub_->publish(empty_path);
}

// 主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CornRowDetectorProjection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}