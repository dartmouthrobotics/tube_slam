// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// ROS2 port with Imu support for underwater sonar SLAM.
//
#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Message filters for Imu sync
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL (for global map accumulation)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace kiss_icp_ros2 {

class OdometryServer : public rclcpp::Node {
public:
    explicit OdometryServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    // Callbacks
    void RegisterFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void PointCloudAndImuCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr &Imu_msg);

    // Publishing helpers
    void PublishOdometry(const Sophus::SE3d &pose,
                         const rclcpp::Time &stamp,
                         const std::string &cloud_frame_id);
    void PublishClouds(const std::vector<Eigen::Vector3d> &frame,
                       const std::vector<Eigen::Vector3d> &keypoints,
                       const rclcpp::Time &stamp,
                       const std::string &cloud_frame_id);
    void AccumulateGlobalMap(const sensor_msgs::msg::PointCloud2 &kiss_map,
                             const std_msgs::msg::Header &header);

    // TF lookup
    Sophus::SE3d LookupTransform(const std::string &target_frame,
                                 const std::string &source_frame) const;

    // KISS-ICP pipeline
    kiss_icp::pipeline::KissICP odometry_;
    kiss_icp::pipeline::KISSConfig config_;

    // ROS 2 subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    // Message filter subscribers (for Imu sync mode)
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Imu>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_filter_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> Imu_filter_;
    std::shared_ptr<Sync> synchronizer_;

    // ROS 2 publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kpoints_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_publisher_;

    // TF
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Path message (accumulated)
    nav_msgs::msg::Path path_msg_;

    // Parameters
    std::string base_frame_ = "";
    std::string odom_frame_ = "odom";
    bool publish_odom_tf_ = true;
    bool publish_debug_clouds_ = true;
    bool downsample_pointcloud_ = false;
    bool use_Imu_ = false;
    int queue_size_ = 100;

    // PCL global map
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_visualization_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_local_map_;
    pcl::VoxelGrid<pcl::PointXYZ> local_map_filter_;
    pcl::VoxelGrid<pcl::PointXYZ> global_map_filter_;
};

}  // namespace kiss_icp_ros2
