// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// ROS2 port with Imu support for underwater sonar SLAM.
//
#include <Eigen/Core>
#include <memory>
#include <utility>
#include <vector>

// KISS-ICP-ROS2
#include "OdometryServer.hpp"
#include "Utils.hpp"

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace kiss_icp_ros2 {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : Node("kiss_icp_odometry", options),
      tf2_buffer_(this->get_clock()),
      tf2_listener_(tf2_buffer_),
      tf_broadcaster_(*this) {
    // ---- Declare and get parameters ----
    base_frame_ = this->declare_parameter<std::string>("base_frame", "");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    publish_odom_tf_ = this->declare_parameter<bool>("publish_odom_tf", true);
    publish_debug_clouds_ = this->declare_parameter<bool>("visualize", true);
    downsample_pointcloud_ = this->declare_parameter<bool>("downsample_pointcloud", false);
    use_Imu_ = this->declare_parameter<bool>("use_Imu", false);

    // KISS-ICP config parameters
    config_.max_range = this->declare_parameter<double>("max_range", 100.0);
    config_.min_range = this->declare_parameter<double>("min_range", 5.0);
    config_.deskew = this->declare_parameter<bool>("deskew", false);
    config_.voxel_size = this->declare_parameter<double>("voxel_size", config_.max_range / 100.0);
    config_.max_points_per_voxel =
        this->declare_parameter<int>("max_points_per_voxel", config_.max_points_per_voxel);
    config_.initial_threshold =
        this->declare_parameter<double>("initial_threshold", config_.initial_threshold);
    config_.min_motion_th =
        this->declare_parameter<double>("min_motion_th", config_.min_motion_th);

    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(this->get_logger(),
                    "max_range is smaller than min_range, setting min_range to 0.0");
        config_.min_range = 0.0;
    }

    // Construct the main KISS-ICP odometry pipeline
    odometry_ = kiss_icp::pipeline::KissICP(config_);

    // ---- Initialize subscribers ----
    if (use_Imu_) {
        // Synchronized point cloud + Imu
        pointcloud_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            this, "pointcloud_topic", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        Imu_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
            this, "Imu_topic", rclcpp::SensorDataQoS().get_rmw_qos_profile());

        synchronizer_ = std::make_shared<Sync>(SyncPolicy(queue_size_),
                                                *pointcloud_filter_, *Imu_filter_);
        synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
        synchronizer_->registerCallback(std::bind(
            &OdometryServer::PointCloudAndImuCallback, this,
            std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Imu mode enabled: subscribing to pointcloud_topic + Imu_topic");
    } else {
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud_topic",
            rclcpp::SensorDataQoS(),
            std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Point cloud only mode: subscribing to pointcloud_topic");
    }

    // ---- Initialize publishers ----
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/kiss/odometry", queue_size_);
    traj_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/kiss/trajectory", queue_size_);
    if (publish_debug_clouds_) {
        frame_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/frame", queue_size_);
        kpoints_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/keypoints", queue_size_);
        map_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/local_map", queue_size_);
        global_map_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/global_map", queue_size_);
    }

    path_msg_.header.frame_id = odom_frame_;

    // ---- Initialize global map (PCL) ----
    if (downsample_pointcloud_) {
        double voxel_size_local = 0.4;
        double voxel_size_global = 1.0;
        local_map_filter_.setLeafSize(voxel_size_local, voxel_size_local, voxel_size_local);
        global_map_filter_.setLeafSize(voxel_size_global, voxel_size_global, voxel_size_global);
    }
    global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_map_visualization_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    latest_local_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS 2 Odometry Node Initialized");
}

// ---- TF Lookup ----
Sophus::SE3d OdometryServer::LookupTransform(const std::string &target_frame,
                                              const std::string &source_frame) const {
    std::string err_msg;
    if (tf2_buffer_._frameExists(source_frame) &&
        tf2_buffer_._frameExists(target_frame) &&
        tf2_buffer_.canTransform(target_frame, source_frame, tf2::TimePointZero, &err_msg)) {
        try {
            auto tf = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }
    RCLCPP_WARN(this->get_logger(), "Failed to find tf between %s and %s. Reason=%s",
                target_frame.c_str(), source_frame.c_str(), err_msg.c_str());
    return {};
}

// ---- Point cloud only callback ----
void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return GetTimestamps(msg);
    }();

    if (points.size() < 30) return;

    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    // Register frame — main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = odometry_.RegisterFrame(points, timestamps);

    // Compute the pose using KISS, ego-centric to the sensor
    const Sophus::SE3d kiss_pose = odometry_.poses().back();

    // If necessary, transform to base_link frame
    const auto pose = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return kiss_pose;
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        return cloud2base * kiss_pose * cloud2base.inverse();
    }();

    PublishOdometry(pose, msg->header.stamp, cloud_frame_id);

    if (publish_debug_clouds_) {
        PublishClouds(frame, keypoints, msg->header.stamp, cloud_frame_id);
    }
}

// ---- Point cloud + Imu synchronized callback ----
void OdometryServer::PointCloudAndImuCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
    const sensor_msgs::msg::Imu::ConstSharedPtr &Imu_msg) {
    const auto cloud_frame_id = cloud_msg->header.frame_id;
    const auto points = PointCloud2ToEigen(cloud_msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return GetTimestamps(cloud_msg);
    }();

    // ---- Imu orientation -> initial guess ----
    // Flip axes to match sonar coordinate frame (from your ROS1 code)
    tf2::Quaternion tf_q_in(
        Imu_msg->orientation.x,
        Imu_msg->orientation.y,
        Imu_msg->orientation.z,
        Imu_msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q_in).getRPY(roll, pitch, yaw);
    // Negate roll and yaw for Imu-to-sonar coordinate flip
    roll = -roll;
    yaw = -yaw;

    tf2::Quaternion tf_q_out;
    tf_q_out.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Quaternion q_out;
    q_out.x = tf_q_out.x();
    q_out.y = tf_q_out.y();
    q_out.z = tf_q_out.z();
    q_out.w = tf_q_out.w();

    // Build SE3d from Imu quaternion (translation = 0, rotation only)
    const Sophus::SE3d odom_pose = tf2::quaternionToSophus(q_out);
    const std::string odom_frame_id = "world";
    const Sophus::SE3d odom2cloud = LookupTransform(odom_frame_, odom_frame_id);

    // Compute initial guess for KISS-ICP registration
    const Sophus::SE3d initial_guess = odom2cloud * odom_pose * odom2cloud.inverse();

    if (points.size() < 30) return;

    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    // Register frame with initial guess from Imu
    const auto &[frame, keypoints] = odometry_.RegisterFrame(points, initial_guess);

    const Sophus::SE3d kiss_pose = odometry_.poses().back();

    const auto pose = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return kiss_pose;
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        return cloud2base * kiss_pose * cloud2base.inverse();
    }();

    PublishOdometry(pose, cloud_msg->header.stamp, cloud_frame_id);

    if (publish_debug_clouds_) {
        PublishClouds(frame, keypoints, cloud_msg->header.stamp, cloud_frame_id);
    }
}

// ---- Publish odometry + tf + path ----
void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
                                     const rclcpp::Time &stamp,
                                     const std::string &cloud_frame_id) {
    // Broadcast tf
    if (publish_odom_tf_) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_.sendTransform(transform_msg);
    }

    // Publish trajectory
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_->publish(path_msg_);

    // Publish odometry
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    odom_publisher_->publish(odom_msg);
}

// ---- Publish debug clouds ----
void OdometryServer::PublishClouds(const std::vector<Eigen::Vector3d> &frame,
                                   const std::vector<Eigen::Vector3d> &keypoints,
                                   const rclcpp::Time &stamp,
                                   const std::string &cloud_frame_id) {
    std_msgs::msg::Header odom_header;
    odom_header.stamp = stamp;
    odom_header.frame_id = odom_frame_;

    const auto kiss_map = odometry_.LocalMap();

    if (!publish_odom_tf_) {
        // Egocentric debugging
        std_msgs::msg::Header cloud_header;
        cloud_header.stamp = stamp;
        cloud_header.frame_id = cloud_frame_id;

        frame_publisher_->publish(*EigenToPointCloud2(frame, cloud_header));
        kpoints_publisher_->publish(*EigenToPointCloud2(keypoints, cloud_header));
        map_publisher_->publish(*EigenToPointCloud2(kiss_map, odom_header));

        auto msg = EigenToPointCloud2(kiss_map, odom_header);
        AccumulateGlobalMap(*msg, odom_header);
        sensor_msgs::msg::PointCloud2 global_cloud_msg;
        global_cloud_msg.header = odom_header;
        pcl::toROSMsg(*global_map_visualization_, global_cloud_msg);
        global_map_publisher_->publish(global_cloud_msg);
        global_map_visualization_->clear();
        return;
    }

    std_msgs::msg::Header cloud_header;
    cloud_header.stamp = stamp;
    cloud_header.frame_id = cloud_frame_id;

    const auto cloud2odom = LookupTransform(odom_frame_, cloud_frame_id);
    frame_publisher_->publish(*EigenToPointCloud2(frame, cloud_header));
    kpoints_publisher_->publish(*EigenToPointCloud2(keypoints, cloud2odom, odom_header));

    if (!base_frame_.empty()) {
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        map_publisher_->publish(*EigenToPointCloud2(kiss_map, cloud2base, odom_header));
        auto msg = EigenToPointCloud2(frame, cloud2odom, odom_header);
        AccumulateGlobalMap(*msg, odom_header);
    } else {
        map_publisher_->publish(*EigenToPointCloud2(kiss_map, odom_header));
        auto msg = EigenToPointCloud2(kiss_map, odom_header);
        AccumulateGlobalMap(*msg, odom_header);
    }

    sensor_msgs::msg::PointCloud2 global_cloud_msg;
    global_cloud_msg.header = odom_header;
    pcl::toROSMsg(*global_map_visualization_, global_cloud_msg);
    global_map_publisher_->publish(global_cloud_msg);
    global_map_visualization_->clear();
}

// ---- Accumulate global map ----
void OdometryServer::AccumulateGlobalMap(const sensor_msgs::msg::PointCloud2 &kiss_map,
                                         const std_msgs::msg::Header &header) {
    pcl::fromROSMsg(kiss_map, *latest_local_map_);

    if (downsample_pointcloud_) {
        local_map_filter_.setInputCloud(latest_local_map_);
        local_map_filter_.filter(*latest_local_map_);
    }

    if (global_map_->empty()) {
        *global_map_ = *latest_local_map_;
    } else {
        *global_map_ += *latest_local_map_;
    }

    if (downsample_pointcloud_) {
        global_map_filter_.setInputCloud(global_map_);
        global_map_filter_.filter(*global_map_visualization_);
    } else {
        *global_map_visualization_ = *global_map_;
    }
    latest_local_map_->clear();
}

}  // namespace kiss_icp_ros2

// ---- Main entry point ----
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<kiss_icp_ros2::OdometryServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
