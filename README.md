# Sonar SLAM ROS2 Workspace

ROS2 workspace for underwater sonar SLAM using KISS-ICP odometry and SimpleLoopClosure pose graph optimization

## Overview

This workspace processes 3D sonar point clouds (Water Linked Sonar 3D-15) through:

1. **KISS-ICP** — Point cloud registration / odometry
2. **SimpleLoopClosure** — ICP-based loop detection with GTSAM pose graph optimization
3. **MapSaverNode** — Accumulates global map and saves PCD + trajectory on shutdown

## Repository Structure

```
sonar_ws/src/
├── kiss-icp/                    # Official KISS-ICP (v1.2.3) + custom additions
│   ├── cpp/kiss_icp/            # Core C++ pipeline (unchanged)
│   ├── ros/                     # ROS2 wrapper
│   │   ├── src/
│   │   │   ├── OdometryServer.cpp   # Official KISS-ICP ROS2 node
│   │   │   └── MapSaverNode.cpp     # [ADDED] Global map accumulator + saver
│   │   ├── config/
│   │   │   ├── config.yaml          # Default LiDAR config
│   │   │   └── sonar_config.yaml    # [ADDED] Sonar-tuned parameters
│   │   ├── launch/
│   │   │   ├── odometry.launch.py   # Official launch
│   │   │   └── sonar_slam.launch.py # [ADDED] All-in-one launch
│   │   └── rviz/
│   │       ├── kiss_icp.rviz        # Default rviz config
│   │       └── sonar_kiss_icp.rviz  # [ADDED] Sonar visualization config
│   └── python/                  # Offline pipeline (unused in ROS2)
│
├── simple_loop_closure/         # ROS2 branch from ashBabu/simple_loop_closure
│   ├── src/simple_loop_closure_node.cpp
│   ├── include/simple_loop_closure/
│   ├── config/simple_loop_closure.yaml
│   ├── launch/simple_loop_closure.launch.py
│   └── thirdparty/nanoflann/    # KD-tree (git submodule)
│
└── README.md
```

## Changes Made

### kiss-icp

| File | Status | Description |
|------|--------|-------------|
| `ros/src/MapSaverNode.cpp` | **Added** | Subscribes to `/kiss/frame` and `/kiss/odometry`, accumulates a global point cloud map, saves `global_map.pcd`, `local_map.pcd`, and `trajectory_tum.txt` on shutdown (Ctrl+C). No PCL dependency. |
| `ros/config/sonar_config.yaml` | **Added** | Tuned for Water Linked 3D-15: `max_range: 10.0`, `min_range: 0.3`, `voxel_size: 0.10`, `deskew: false`, `min_motion_th: 0.05` |
| `ros/launch/sonar_slam.launch.py` | **Added** | All-in-one launch: KISS-ICP + SimpleLoopClosure + MapSaver + RViz + bag playback. Supports `use_loop_closure:=true/false` |
| `ros/rviz/sonar_kiss_icp.rviz` | **Added** | Custom rviz config: local map pixel size 1.2, color (246, 245, 244), fixed frame `odom` |
| `ros/CMakeLists.txt` | **Modified** | Added `map_saver_node` target, `find_package(Eigen3)`, `cmake_policy(SET CMP0144 NEW)` |
| `ros/package.xml` | **Modified** | Added `pcl_conversions` dependency |

### simple_loop_closure

| File | Status | Description |
|------|--------|-------------|
| `include/.../simple_loop_closure_node.hpp` | **Modified** | Changed `pcl::PointXYZI` → `pcl::PointXYZ` (KISS-ICP publishes XYZ only) |
| `src/simple_loop_closure_node.cpp` | **Modified** | Same PointXYZI → PointXYZ change. Fixed sign comparison warnings. |
| `CMakeLists.txt` | **Modified** | Added `cmake_policy(SET CMP0144 NEW)` to suppress FLANN warning |

## Dependencies

```bash
# ROS2 Jazzy packages
sudo apt install \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-eigen \
  ros-jazzy-message-filters \
  ros-jazzy-pcl-conversions \
  ros-jazzy-visualization-msgs \
  ros-jazzy-image-transport-plugins

# GTSAM (for loop closure)
sudo apt install libgtsam-dev libgtsam-unstable-dev

# MPI (required for PCL/VTK on Ubuntu 24.04)
sudo apt install libopenmpi-dev
```

## Build

```bash
cd ~/Desktop/sonar_ws
colcon build
source install/setup.bash
```

## Usage

### Run everything (KISS-ICP + Loop Closure + Map Saver + RViz + Bag)

```bash
ros2 launch kiss_icp sonar_slam.launch.py
```

### Without loop closure

```bash
ros2 launch kiss_icp sonar_slam.launch.py use_loop_closure:=false
```

### Custom bag file or playback rate

```bash
ros2 launch kiss_icp sonar_slam.launch.py \
  bagfile:=/path/to/your.db3 \
  rate:=2.0
```

### KISS-ICP only 

```bash
ros2 launch kiss_icp odometry.launch.py \
  topic:=/sonar_3d/point_cloud \
  config_file:=$(ros2 pkg prefix kiss_icp)/share/kiss_icp/config/sonar_config.yaml \
  bagfile:=/path/to/your.db3
```

## Output

On Ctrl+C, the map saver writes to `~/Desktop/kiss_icp_results/`:

| File | Description |
|------|-------------|
| `global_map.pcd` | Cumulative map from all registered frames |
| `local_map.pcd` | Last KISS-ICP sliding window map |
| `trajectory_tum.txt` | Full trajectory in TUM format (`timestamp tx ty tz qx qy qz qw`) |

### View results

```bash
# View point cloud
pcl_viewer ~/Desktop/kiss_icp_results/global_map.pcd

# Or with CloudCompare
sudo snap install cloudcompare
cloudcompare.CloudCompare ~/Desktop/kiss_icp_results/global_map.pcd
```

## Topics

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/sonar_3d/point_cloud` | PointCloud2 | Bag | Raw sonar point cloud |
| `/kiss/odometry` | Odometry | KISS-ICP | Estimated odometry |
| `/kiss/frame` | PointCloud2 | KISS-ICP | Current registered frame |
| `/kiss/local_map` | PointCloud2 | KISS-ICP | Sliding window local map |
| `/kiss/keypoints` | PointCloud2 | KISS-ICP | Extracted keypoints |
| `/pgo_odom` | Odometry | Loop Closure | Pose-graph optimized odometry |
| `/pgo_map_cloud` | PointCloud2 | Loop Closure | Optimized map |
| `/vis_pose_graph` | MarkerArray | Loop Closure | Pose graph visualization |

## Sonar Config Parameters

```yaml
data:
  deskew: false       # Sonar doesn't need motion deskew
  max_range: 10.0     # Water Linked 3D-15 effective range
  min_range: 0.3      # Reject near-field noise

mapping:
  voxel_size: 0.10    # Fine voxels for short-range sonar
  max_points_per_voxel: 20

adaptive_threshold:
  initial_threshold: 2.0
  min_motion_th: 0.05  # Lower for slow underwater motion
```

## Loop Closure Parameters 

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `keyframe_dist_th` | 1.0 m | Reduce keyframe density |
| `loop_search_time_diff_th` | 60.0 s | Must be 60s+ apart |
| `loop_search_dist_diff_th` | 50.0 m | Must travel 50m+ before matching |
| `loop_search_frame_interval` | 3 | Check every 3rd frame |
| `fitness_score_th` | 0.15 | Strict ICP match threshold |

