// Map saver node for KISS-ICP
// Accumulates global map from /kiss/frame with voxel dedup,
// saves binary PCD + trajectory on shutdown (Ctrl+C).

#include <csignal>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <atomic>
#include <cmath>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Simple spatial hash for voxel deduplication (no PCL needed)
struct VoxelKey {
    int64_t x, y, z;
    bool operator==(const VoxelKey &o) const { return x == o.x && y == o.y && z == o.z; }
};
struct VoxelKeyHash {
    size_t operator()(const VoxelKey &k) const {
        size_t h = std::hash<int64_t>()(k.x);
        h ^= std::hash<int64_t>()(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int64_t>()(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

class MapSaverNode : public rclcpp::Node {
public:
    MapSaverNode() : Node("map_saver_node") {
        save_directory_ = this->declare_parameter<std::string>(
            "save_directory", "/home/xiaoming-zhao/Desktop/kiss_icp_results/");
        global_voxel_size_ = this->declare_parameter<double>(
            "global_voxel_size", 0.05);  // 5cm voxel — keeps one point per voxel

        // /kiss/frame: registered frame points (already in odom frame)
        frame_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/kiss/frame", rclcpp::SensorDataQoS(),
            std::bind(&MapSaverNode::frameCallback, this, std::placeholders::_1));

        // /kiss/local_map: sliding window map (keep latest snapshot)
        map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/kiss/local_map", rclcpp::SensorDataQoS(),
            std::bind(&MapSaverNode::mapCallback, this, std::placeholders::_1));

        // /kiss/odometry: poses for trajectory export
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/kiss/odometry", 10,
            std::bind(&MapSaverNode::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "Map saver ready (voxel=%.3fm). Output: %s",
            global_voxel_size_, save_directory_.c_str());
    }

    void saveAll() {
        if (global_points_.empty() && !has_local_map_ && trajectory_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No data to save.");
            return;
        }

        std::filesystem::create_directories(save_directory_);

        if (!global_points_.empty()) {
            std::string path = save_directory_ + "global_map.pcd";
            saveBinaryPCD(global_points_, path);
            RCLCPP_INFO(this->get_logger(), "Saved global map (%zu points) to: %s",
                         global_points_.size(), path.c_str());
        }

        if (has_local_map_) {
            std::string path = save_directory_ + "local_map.pcd";
            saveBinaryPCD(local_map_points_, path);
            RCLCPP_INFO(this->get_logger(), "Saved local map (%zu points) to: %s",
                         local_map_points_.size(), path.c_str());
        }

        if (!trajectory_.empty()) {
            std::string traj_path = save_directory_ + "trajectory_tum.txt";
            std::ofstream f(traj_path);
            f << "# timestamp tx ty tz qx qy qz qw\n";
            for (const auto &p : trajectory_) {
                f << std::fixed << std::setprecision(6)
                  << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << " "
                  << p[4] << " " << p[5] << " " << p[6] << " " << p[7] << "\n";
            }
            f.close();
            RCLCPP_INFO(this->get_logger(), "Saved trajectory (%zu poses) to: %s",
                         trajectory_.size(), traj_path.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "All results saved.");
    }

private:
    void frameCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

        double inv = 1.0 / global_voxel_size_;
        for (; ix != ix.end(); ++ix, ++iy, ++iz) {
            float x = *ix, y = *iy, z = *iz;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            VoxelKey key{
                static_cast<int64_t>(std::floor(x * inv)),
                static_cast<int64_t>(std::floor(y * inv)),
                static_cast<int64_t>(std::floor(z * inv))
            };

            if (occupied_.find(key) == occupied_.end()) {
                occupied_.insert(key);
                global_points_.push_back({x, y, z});
            }
        }
        frame_count_++;
        if (frame_count_ % 500 == 0) {
            RCLCPP_INFO(this->get_logger(), "Global map: %zu pts (%d frames)",
                         global_points_.size(), frame_count_);
        }
    }

    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        local_map_points_.clear();
        sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
        for (; ix != ix.end(); ++ix, ++iy, ++iz) {
            if (std::isfinite(*ix) && std::isfinite(*iy) && std::isfinite(*iz))
                local_map_points_.push_back({*ix, *iy, *iz});
        }
        has_local_map_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        trajectory_.push_back({
            rclcpp::Time(msg->header.stamp).seconds(),
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        });
    }

    // Binary PCD: ~100x faster than ASCII for 20M+ points
    void saveBinaryPCD(const std::vector<std::array<float, 3>> &points,
                       const std::string &filename) {
        std::ofstream f(filename, std::ios::binary);
        f << "# .PCD v0.7 - Point Cloud Data file format\n"
          << "VERSION 0.7\n"
          << "FIELDS x y z\n"
          << "SIZE 4 4 4\n"
          << "TYPE F F F\n"
          << "COUNT 1 1 1\n"
          << "WIDTH " << points.size() << "\n"
          << "HEIGHT 1\n"
          << "VIEWPOINT 0 0 0 1 0 0 0\n"
          << "POINTS " << points.size() << "\n"
          << "DATA binary\n";
        // Single write for all point data
        f.write(reinterpret_cast<const char*>(points.data()),
                static_cast<std::streamsize>(points.size() * sizeof(std::array<float, 3>)));
        f.close();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr frame_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_;
    std::vector<std::array<float, 3>> global_points_;
    double global_voxel_size_;

    std::vector<std::array<float, 3>> local_map_points_;
    bool has_local_map_ = false;

    std::vector<std::array<double, 8>> trajectory_;
    std::string save_directory_;
    int frame_count_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapSaverNode>();

    // Custom signal handler — ensures saveAll() runs before exit
    std::signal(SIGINT, [](int) { rclcpp::shutdown(); });
    std::signal(SIGTERM, [](int) { rclcpp::shutdown(); });

    rclcpp::spin(node);

    // After spin exits, save everything
    RCLCPP_INFO(node->get_logger(), "Shutting down — saving results...");
    node->saveAll();

    return 0;
}