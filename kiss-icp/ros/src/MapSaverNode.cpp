// Map saver node for KISS-ICP
// Accumulates all registered frames into a global map and saves on shutdown.
// Also saves trajectory in TUM format.

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <mutex>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class MapSaverNode : public rclcpp::Node {
public:
    MapSaverNode() : Node("map_saver_node") {
        save_directory_ = this->declare_parameter<std::string>(
            "save_directory", "/home/xiaoming-zhao/Desktop/kiss_icp_results/");
        voxel_size_ = this->declare_parameter<double>("voxel_size", 0.1);

        // Subscribe to registered frame (already transformed to odom frame by KISS-ICP)
        frame_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/kiss/frame", rclcpp::SensorDataQoS(),
            std::bind(&MapSaverNode::frameCallback, this, std::placeholders::_1));

        // Also keep local map for reference
        map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/kiss/local_map", rclcpp::SensorDataQoS(),
            std::bind(&MapSaverNode::localMapCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/kiss/odometry", 10,
            std::bind(&MapSaverNode::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Map saver initialized. Accumulating global map...");
        RCLCPP_INFO(this->get_logger(), "Will save to: %s", save_directory_.c_str());
    }

    ~MapSaverNode() {
        saveResults();
    }

private:
    struct Point3f {
        float x, y, z;
    };

    void frameCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Get the latest pose to transform frame points to odom frame
        Eigen::Affine3d pose;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (trajectory_.empty()) return;
            auto &p = trajectory_.back();
            Eigen::Quaterniond q(p[7], p[4], p[5], p[6]); // qw, qx, qy, qz
            pose = Eigen::Affine3d::Identity();
            pose.translate(Eigen::Vector3d(p[1], p[2], p[3]));
            pose.rotate(q);
        }

        // Extract points and transform to odom frame
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        std::lock_guard<std::mutex> lock(mtx_);
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z))
                continue;

            Eigen::Vector3d pt_local(*iter_x, *iter_y, *iter_z);
            Eigen::Vector3d pt_global = pose * pt_local;

            global_map_.push_back({
                static_cast<float>(pt_global.x()),
                static_cast<float>(pt_global.y()),
                static_cast<float>(pt_global.z())
            });
        }
        frame_count_++;

        if (frame_count_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Accumulated %d frames, %zu points total",
                         frame_count_, global_map_.size());
        }
    }

    void localMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        latest_local_map_ = *msg;
        has_local_map_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double stamp = rclcpp::Time(msg->header.stamp).seconds();

        std::lock_guard<std::mutex> lock(mtx_);
        trajectory_.push_back({stamp, x, y, z, qx, qy, qz, qw});
    }

    void savePCD(const std::vector<Point3f> &points, const std::string &filename) {
        std::ofstream f(filename);
        f << "# .PCD v0.7 - Point Cloud Data file format\n";
        f << "VERSION 0.7\n";
        f << "FIELDS x y z\n";
        f << "SIZE 4 4 4\n";
        f << "TYPE F F F\n";
        f << "COUNT 1 1 1\n";
        f << "WIDTH " << points.size() << "\n";
        f << "HEIGHT 1\n";
        f << "VIEWPOINT 0 0 0 1 0 0 0\n";
        f << "POINTS " << points.size() << "\n";
        f << "DATA ascii\n";
        f << std::fixed << std::setprecision(6);
        for (const auto &p : points) {
            f << p.x << " " << p.y << " " << p.z << "\n";
        }
        f.close();
    }

    void savePCDFromMsg(const sensor_msgs::msg::PointCloud2 &cloud, const std::string &filename) {
        std::vector<Point3f> points;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
                points.push_back({*iter_x, *iter_y, *iter_z});
            }
        }
        savePCD(points, filename);
    }

    void saveResults() {
        std::lock_guard<std::mutex> lock(mtx_);

        if (global_map_.empty() && trajectory_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No data to save.");
            return;
        }

        std::filesystem::create_directories(save_directory_);

        // Save cumulative global map
        if (!global_map_.empty()) {
            std::string pcd_path = save_directory_ + "global_map.pcd";
            savePCD(global_map_, pcd_path);
            RCLCPP_INFO(this->get_logger(), "Saved global map (%zu points) to: %s",
                         global_map_.size(), pcd_path.c_str());
        }

        // Save last local map
        if (has_local_map_) {
            std::string local_path = save_directory_ + "local_map.pcd";
            savePCDFromMsg(latest_local_map_, local_path);
            RCLCPP_INFO(this->get_logger(), "Saved local map to: %s", local_path.c_str());
        }

        // Save trajectory as TUM format
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

        RCLCPP_INFO(this->get_logger(), "All results saved to: %s", save_directory_.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr frame_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::mutex mtx_;
    std::vector<Point3f> global_map_;
    sensor_msgs::msg::PointCloud2 latest_local_map_;
    bool has_local_map_ = false;
    std::vector<std::array<double, 8>> trajectory_;
    std::string save_directory_;
    double voxel_size_;
    int frame_count_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}