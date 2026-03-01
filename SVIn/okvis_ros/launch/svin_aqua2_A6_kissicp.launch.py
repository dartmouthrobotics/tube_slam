import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    okvis_config_rel = LaunchConfiguration('okvis_config').perform(context)
    launch_file_dir = ThisLaunchFileDir().perform(context)
    abs_okvis_config_path = os.path.abspath(
        os.path.join(launch_file_dir, okvis_config_rel)
    )
    print(f"\n[INFO] SVIn config: {abs_okvis_config_path}\n")

    # SVIn: OKVIS VIO 
    okvis_node = Node(
        package='okvis_ros',
        executable='okvis_node',
        name='okvis_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_filename': abs_okvis_config_path,
            'mesh_file': 'firefly.dae',
        }],
        remappings=[
            ('/camera0', '/cam0/image_raw'),
            ('/camera1', '/cam1/image_raw'),
            ('/imu', '/a6/imu/filtered_data'),
            ('/depth', '/a6/depth_stamped'),
        ],
    )

    # SVIn: Pose Graph 
    pose_graph_node = Node(
        package='pose_graph',
        executable='pose_graph_node',
        name='pose_graph_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': abs_okvis_config_path,
        }],
    )

    return [okvis_node, pose_graph_node]


def generate_launch_description():
    kiss_icp_pkg = FindPackageShare('kiss_icp')

    kiss_icp_config_default = os.path.join(
        get_package_share_directory('kiss_icp'), 'config', 'sonar_config.yaml'
    )

    # ---- Launch Arguments ----
    args = [
        DeclareLaunchArgument(
            'okvis_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('okvis_ros'), 'config',
                'config_aqua2_A6_BBDOS26_1280_720.yaml',
            ]),
        ),
        DeclareLaunchArgument('kiss_icp_config', default_value=kiss_icp_config_default),
        DeclareLaunchArgument(
            'bagfile',
            default_value='/home/xiaoming-zhao/Desktop/synchronized_merge_bag_01-25__15_23_w_sonar_0.db3',
        ),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('visualize', default_value='true'),
        DeclareLaunchArgument('use_loop_closure', default_value='true'),
        DeclareLaunchArgument('external_odom_topic', default_value='/okvis_odometry'),
        DeclareLaunchArgument(
            'save_directory',
            default_value='/home/xiaoming-zhao/Desktop/kiss_icp_results/',
        ),
    ]

    # ---- SVIn: Stereo Sync  ----
    stereo_sync_node = Node(
        package='okvis_ros',
        executable='stereo_sync',
        name='stereo_sync',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_filename': LaunchConfiguration('okvis_config'),
            'left_img_topic': '/a6/camera/left/image_raw',
            'right_img_topic': '/a6/camera/right/image_raw',
            'compressed': True,
        }],
    )

    # ---- Sonar + SVIn loose coupling ----
    kiss_icp_node = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        output='screen',
        remappings=[
            ('pointcloud_topic', '/sonar_3d/point_cloud'),
        ],
        parameters=[
            {
                'base_frame': '',
                'lidar_odom_frame': 'odom',
                'publish_odom_tf': True,
                'invert_odom_tf': True,
                'publish_debug_clouds': True,
                'use_sim_time': True,
                'position_covariance': 0.1,
                'orientation_covariance': 0.1,
                'use_external_odom': True,
                'external_odom_topic': LaunchConfiguration('external_odom_topic'),
            },
            LaunchConfiguration('kiss_icp_config'),
        ],
    )

    # ---- Simple Loop Closure  ----
    loop_closure_node = Node(
        package='simple_loop_closure',
        executable='simple_loop_closure_node',
        name='simple_loop_closure',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_loop_closure')),
        remappings=[
            ('/cloud', '/kiss/frame'),
            ('/odometry', '/kiss/odometry'),
        ],
        parameters=[{
            'use_sim_time': True,
            'mapped_cloud': False,
            'time_stamp_tolerance': 0.01,
            'keyframe_dist_th': 1.0,
            'keyframe_angular_dist_th': 0.5,
            'loop_search_time_diff_th': 60.0,
            'loop_search_dist_diff_th': 50.0,
            'loop_search_angular_dist_th': 3.1415,
            'loop_search_frame_interval': 3,
            'search_radius': 10.0,
            'target_frame_num': 20,
            'target_voxel_leaf_size': 0.0,
            'source_voxel_leaf_size': 0.0,
            'vis_map_voxel_leaf_size': 0.4,
            'fitness_score_th': 0.15,
            'vis_map_cloud_frame_interval': 3,
        }],
    )

    # ---- Map Saver ----
    map_saver_node = Node(
        package='kiss_icp',
        executable='map_saver_node',
        name='map_saver_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'save_directory': LaunchConfiguration('save_directory'),
        }],
    )

    # ---- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([kiss_icp_pkg, 'rviz', 'kiss_icp.rviz']),
        ],
        condition=IfCondition(LaunchConfiguration('visualize')),
    )

    # ---- Bag Playback ----
    bag_playback = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play',
                    '--rate', LaunchConfiguration('rate'),
                    LaunchConfiguration('bagfile'),
                    '--clock', '200',
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription(
        args + [
            stereo_sync_node,
            OpaqueFunction(function=launch_setup),
            kiss_icp_node,
            loop_closure_node,
            map_saver_node,
            rviz_node,
            bag_playback,
        ]
    )