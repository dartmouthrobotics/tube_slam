# Underwater Sonar SLAM with Visual-Inertial Loose Coupling

ROS2 workspace for underwater SLAM combining SVIn with KISS-ICP sonar odometry through loose coupling, plus ICP-based loop closure

## Repository Structure

```
tube_slam/
├── kiss-icp/                        
│   ├── cpp/kiss_icp/                
│   │   ├── pipeline/KissICP.cpp     
│   │   └── core/                    
│   ├── ros/                        
│   │   ├── src/
│   │   │   ├── OdometryServer.cpp   # Added SVIn odom subscriber + delta override
│   │   │   ├── OdometryServer.hpp   # Added loose coupling member variables
│   │   │   └── MapSaverNode.cpp     # Global map accumulator + saver
│   │   ├── config/
│   │   │   ├── config.yaml         
│   │   │   └── sonar_config.yaml    # Sonar-tuned parameters
│   │   ├── launch/
│   │   │   ├── odometry.launch.py   # Official launch
│   │   │   └── sonar.launch.py      # Sonar launch with loose coupling params
│   │   └── rviz/
│   │       └── kiss_icp.rviz        # frame=odom, added SVIn odom display
│   └── python/                     
│
├── SVIn/                            # SVIn stereo visual-inertial navigation
│   ├── config/                      # OKVIS config files per platform
│   │   └── config_aqua2_A6_BBDOS26_1280_720.yaml  # Aqua A6 config
│   ├── okvis_ros/                   # ROS2 wrapper for OKVIS
│   │   ├── launch/
│   │   │   ├── svin_aqua2_A6_uw.launch.py         
│   │   │   └── svin_aqua2_A6_kissicp.launch.py         # SVIn + kiss-icp launch
│   │   └── src/
│   │       ├── okvis_node.cpp       # VIO node, /okvis/odometry
│   │       ├── stereo_sync.cpp      # Decompress + sync stereo images
│   │       └── uncompress_image.cpp # Single image decompression
│   └── pose_graph/                  # SVIn visual loop closure
│       └── src/pose_graph_node.cpp
│
├── simple_loop_closure/             # ICP-based sonar loop closure
│   ├── src/simple_loop_closure_node.cpp
│   ├── config/simple_loop_closure.yaml
│   └── thirdparty/nanoflann/        # KD-tree
│
└── README.md
```

## Dependencies

```bash
# ROS2 Jazzy
sudo apt install \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-eigen \
  ros-jazzy-message-filters \
  ros-jazzy-pcl-conversions \
  ros-jazzy-visualization-msgs \
  ros-jazzy-image-transport-plugins

# GTSAM 
sudo apt install libgtsam-dev libgtsam-unstable-dev

# MPI 
sudo apt install libopenmpi-dev
```

## Build

```bash
cd ~/Downloads/tube_slam
colcon build
source install/setup.bash
```

## Usage

### All-in-one: SVIn + KISS-ICP + Loop Closure



```bash
ros2 launch okvis_ros svin_aqua2_A6_kissicp.launch.py
```

With custom parameters:

```bash
ros2 launch okvis_ros svin_aqua2_A6_kissicp.launch.py \
  bagfile:=/path/to/bag \
  rate:=0.5 \
  visualize:=false \
  use_loop_closure:=false
```

### KISS-ICP only 

```bash
ros2 launch kiss_icp sonar.launch.py use_external_odom:=false
```

### KISS-ICP with loose coupling 

```bash
# Terminal 1: SVIn
ros2 launch okvis_ros svin_aqua2_A6_uw.launch.py

# Terminal 2: KISS-ICP consuming SVIn odometry
ros2 launch kiss_icp sonar.launch.py use_external_odom:=true
```

## Launch Arguments

### svin_aqua2_A6_kissicp.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `bagfile` | `synchronized_merge_bag_01-25__15_23_w_sonar_0.db3` | ROS2 bag path |
| `rate` | `1.0` | Bag playback rate |
| `visualize` | `true` | Launch RViz |
| `use_loop_closure` | `true` | Enable sonar loop closure |
| `external_odom_topic` | `/okvis/odometry` | SVIn odometry topic |
| `save_directory` | `~/Desktop/kiss_icp_results/` | Map save directory |
| `okvis_config` | `config_aqua2_A6_BBDOS26_1280_720.yaml` | SVIn config |
| `kiss_icp_config` | `sonar_config.yaml` | KISS-ICP config |

### sonar.launch.py 

| Argument | Default | Description |
|----------|---------|-------------|
| `use_external_odom` | `false` | Enable SVIn loose coupling |
| `external_odom_topic` | `/okvis/odometry` | External odometry topic |
| `bagfile` | *(path)* | ROS2 bag path |
| `rate` | `1.0` | Bag playback rate |
| `use_loop_closure` | `true` | Enable loop closure |

## Topics

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/sonar_3d/point_cloud` | PointCloud2 | Bag | Raw sonar point cloud |
| `/okvis/odometry` | Odometry | SVIn | Visual-inertial odometry |
| `/kiss/odometry` | Odometry | KISS-ICP | Sonar odometry (with SVIn-informed initial guess) |
| `/kiss/frame` | PointCloud2 | KISS-ICP | Current registered sonar frame |
| `/kiss/local_map` | PointCloud2 | KISS-ICP | Sliding window local map |
| `/kiss/keypoints` | PointCloud2 | KISS-ICP | Extracted keypoints |
| `/pgo_odom` | Odometry | Loop Closure | Pose-graph optimized odometry |
| `/pgo_map_cloud` | PointCloud2 | Loop Closure | Optimized map |

## Output

On Ctrl+C, outputs are saved:

**KISS-ICP MapSaver** to `~/Desktop/kiss_icp_results/`:

| File | Description |
|------|-------------|
| `global_map.pcd` | Cumulative map from all registered frames |
| `local_map.pcd` | Last KISS-ICP sliding window map |
| `trajectory_tum.txt` | Trajectory in TUM format (`timestamp tx ty tz qx qy qz qw`) |

**SVIn Pose Graph** to `SVIn/pose_graph/svin_results/`:

| File | Description |
|------|-------------|
| `svin_YYYY_MM_DD_HH_MM_SS.txt` | SVIn VIO trajectory |
| `svin_YYYY_MM_DD_HH_MM_SS.ply` | SVIn trajectory as PLY |

## Configuration

### Sonar Parameters (sonar_config.yaml)

```yaml
data:
  deskew: false       # no need motion deskew
  max_range: 10.0     # Water Linked 3D-15 effective range
  min_range: 0.3      # reject near-field noise

mapping:
  voxel_size: 0.10    
  max_points_per_voxel: 20

adaptive_threshold:
  initial_threshold: 2.0
  min_motion_th: 0.05  # lower for slow motion
```

### Loop Closure Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `keyframe_dist_th` | 1.0 m | Reduce keyframe density |
| `loop_search_time_diff_th` | 60.0 s | Must be 60s+ apart |
| `loop_search_dist_diff_th` | 50.0 m | Must travel 50m+ before matching |
| `loop_search_frame_interval` | 3 | Check every 3rd frame |
| `fitness_score_th` | 0.15 | Strict ICP match threshold |