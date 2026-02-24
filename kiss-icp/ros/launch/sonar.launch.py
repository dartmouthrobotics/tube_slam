import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    kiss_icp_pkg = FindPackageShare("kiss_icp")
    kiss_icp_config = os.path.join(
        get_package_share_directory("kiss_icp"), "config", "sonar_config.yaml"
    )

    try:
        slc_config = os.path.join(
            get_package_share_directory("simple_loop_closure"), "config", "simple_loop_closure.yaml"
        )
    except Exception:
        slc_config = ""

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    visualize = LaunchConfiguration("visualize", default="true")
    use_loop_closure = LaunchConfiguration("use_loop_closure", default="true")
    bagfile = LaunchConfiguration("bagfile",
        default="/home/xiaoming-zhao/Desktop/synchronized_merge_bag_01-25__15_23_w_sonar_0.db3")
    rate = LaunchConfiguration("rate", default="1.0")
    config_file = LaunchConfiguration("config_file", default=kiss_icp_config)

    return LaunchDescription([
        DeclareLaunchArgument("bagfile", default_value=bagfile),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("visualize", default_value="true"),
        DeclareLaunchArgument("use_loop_closure", default_value="true"),
        DeclareLaunchArgument("config_file", default_value=kiss_icp_config),

        # ---- KISS-ICP Odometry ----
        Node(
            package="kiss_icp",
            executable="kiss_icp_node",
            name="kiss_icp_node",
            output="screen",
            remappings=[
                ("pointcloud_topic", "/sonar_3d/point_cloud"),
            ],
            parameters=[
                {
                    "base_frame": "",
                    "lidar_odom_frame": "odom",
                    "publish_odom_tf": True,
                    "invert_odom_tf": True,
                    "publish_debug_clouds": True,
                    "use_sim_time": use_sim_time,
                    "position_covariance": 0.1,
                    "orientation_covariance": 0.1,
                },
                config_file,
            ],
        ),

        # ---- Simple Loop Closure ----
        Node(
            package="simple_loop_closure",
            executable="simple_loop_closure_node",
            name="simple_loop_closure",
            output="screen",
            condition=IfCondition(use_loop_closure),
            remappings=[
                ("/cloud", "/kiss/frame"),
                ("/odometry", "/kiss/odometry"),
            ],
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "mapped_cloud": False,
                    "time_stamp_tolerance": 0.01,
                    "keyframe_dist_th": 1.0,
                    "keyframe_angular_dist_th": 0.5,
                    "loop_search_time_diff_th": 60.0,
                    "loop_search_dist_diff_th": 50.0,
                    "loop_search_angular_dist_th": 3.1415,
                    "loop_search_frame_interval": 3,
                    "search_radius": 10.0,
                    "target_frame_num": 20,
                    "target_voxel_leaf_size": 0.0,
                    "source_voxel_leaf_size": 0.0,
                    "vis_map_voxel_leaf_size": 0.4,
                    "fitness_score_th": 0.15,
                    "vis_map_cloud_frame_interval": 3,
                },
            ],
        ),

        # ---- Map Saver ----
        Node(
            package="kiss_icp",
            executable="map_saver_node",
            name="map_saver_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "save_directory": "/home/xiaoming-zhao/Desktop/kiss_icp_results/",
            }],
        ),

        # ---- RViz2 ----
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution([kiss_icp_pkg, "rviz", "kiss_icp.rviz"]),
            ],
            condition=IfCondition(visualize),
        ),

        # ---- Bag Playback ----
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "bag", "play",
                        "--rate", LaunchConfiguration("rate"),
                        LaunchConfiguration("bagfile"),
                        "--clock", "1000.0",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])