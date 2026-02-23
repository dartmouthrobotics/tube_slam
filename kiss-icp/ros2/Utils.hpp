// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// ROS2 port of utility functions.
//
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <cstring>
#include <optional>
#include <regex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include <sophus/se3.hpp>

namespace tf2 {

// ---- Sophus <-> ROS2 msg conversions ----

inline Sophus::SE3d transformToSophus(const geometry_msgs::msg::TransformStamped &transform) {
    const auto &t = transform.transform.translation;
    const auto &q = transform.transform.rotation;
    return Sophus::SE3d(
        Sophus::SE3d::QuaternionType(q.w, q.x, q.y, q.z),
        Sophus::SE3d::Point(t.x, t.y, t.z));
}

inline geometry_msgs::msg::Transform sophusToTransform(const Sophus::SE3d &pose) {
    geometry_msgs::msg::Transform t;
    t.translation.x = pose.translation().x();
    t.translation.y = pose.translation().y();
    t.translation.z = pose.translation().z();
    auto q = pose.unit_quaternion();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
    t.rotation.w = q.w();
    return t;
}

inline geometry_msgs::msg::Pose sophusToPose(const Sophus::SE3d &pose) {
    geometry_msgs::msg::Pose p;
    p.position.x = pose.translation().x();
    p.position.y = pose.translation().y();
    p.position.z = pose.translation().z();
    auto q = pose.unit_quaternion();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}

inline Sophus::SE3d quaternionToSophus(const geometry_msgs::msg::Quaternion &q) {
    return Sophus::SE3d(
        Sophus::SE3d::QuaternionType(q.w, q.x, q.y, q.z),
        Sophus::SE3d::Point(0.0, 0.0, 0.0));
}

}  // namespace tf2

namespace kiss_icp_ros2 {
namespace utils {

// ---- PointCloud2 <-> Eigen conversions ----

inline std::vector<Eigen::Vector3d> PointCloud2ToEigen(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z))
            continue;
        points.emplace_back(*iter_x, *iter_y, *iter_z);
    }
    return points;
}

// Overload for ConstSharedPtr
inline std::vector<Eigen::Vector3d> PointCloud2ToEigen(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z))
            continue;
        points.emplace_back(*iter_x, *iter_y, *iter_z);
    }
    return points;
}

inline std::unique_ptr<sensor_msgs::msg::PointCloud2> EigenToPointCloud2(
    const std::vector<Eigen::Vector3d> &points,
    const std_msgs::msg::Header &header) {
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    msg->header = header;
    msg->height = 1;
    msg->width = static_cast<uint32_t>(points.size());
    msg->is_dense = true;
    msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(*msg);
    modifier.setPointCloud2Fields(
        3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    for (size_t i = 0; i < points.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = static_cast<float>(points[i].x());
        *iter_y = static_cast<float>(points[i].y());
        *iter_z = static_cast<float>(points[i].z());
    }
    return msg;
}

// Overload with transform applied
inline std::unique_ptr<sensor_msgs::msg::PointCloud2> EigenToPointCloud2(
    const std::vector<Eigen::Vector3d> &points,
    const Sophus::SE3d &transform,
    const std_msgs::msg::Header &header) {
    std::vector<Eigen::Vector3d> transformed_points;
    transformed_points.reserve(points.size());
    for (const auto &pt : points) {
        transformed_points.push_back(transform * pt);
    }
    return EigenToPointCloud2(transformed_points, header);
}

inline std::vector<double> GetTimestamps(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    // Try to find a "time" or "t" or "timestamp" field
    auto has_field = [&msg](const std::string &name) {
        return std::any_of(msg->fields.begin(), msg->fields.end(),
                           [&name](const auto &f) { return f.name == name; });
    };

    std::string time_field;
    if (has_field("t"))
        time_field = "t";
    else if (has_field("time"))
        time_field = "time";
    else if (has_field("timestamp"))
        time_field = "timestamp";
    else
        return {};

    std::vector<double> timestamps;
    timestamps.reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> iter(*msg, time_field);
    for (; iter != iter.end(); ++iter) {
        timestamps.push_back(static_cast<double>(*iter));
    }

    // Normalize timestamps to [0, 1]
    if (!timestamps.empty()) {
        const double min_t = *std::min_element(timestamps.begin(), timestamps.end());
        const double max_t = *std::max_element(timestamps.begin(), timestamps.end());
        const double range = max_t - min_t;
        if (range > 0.0) {
            for (auto &t : timestamps) {
                t = (t - min_t) / range;
            }
        }
    }
    return timestamps;
}

// Overload for ConstSharedPtr
inline std::vector<double> GetTimestamps(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    auto has_field = [&msg](const std::string &name) {
        return std::any_of(msg->fields.begin(), msg->fields.end(),
                           [&name](const auto &f) { return f.name == name; });
    };

    std::string time_field;
    if (has_field("t"))
        time_field = "t";
    else if (has_field("time"))
        time_field = "time";
    else if (has_field("timestamp"))
        time_field = "timestamp";
    else
        return {};

    std::vector<double> timestamps;
    timestamps.reserve(msg->height * msg->width);
    sensor_msgs::PointCloud2ConstIterator<float> iter(*msg, time_field);
    for (; iter != iter.end(); ++iter) {
        timestamps.push_back(static_cast<double>(*iter));
    }

    if (!timestamps.empty()) {
        const double min_t = *std::min_element(timestamps.begin(), timestamps.end());
        const double max_t = *std::max_element(timestamps.begin(), timestamps.end());
        const double range = max_t - min_t;
        if (range > 0.0) {
            for (auto &t : timestamps) {
                t = (t - min_t) / range;
            }
        }
    }
    return timestamps;
}

}  // namespace utils
}  // namespace kiss_icp_ros2
