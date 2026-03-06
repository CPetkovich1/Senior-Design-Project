import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths to packages and files
    pkg_description = get_package_share_directory('robot_description')
    pkg_bringup = get_package_share_directory('robot_bringup')
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # 2. Start Robot State Publisher (Your URDF)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 3. Start Camera Node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link_optical',
        }]
    )

    # 4. Start RTAB-Map (SLAM + Visual Odometry)
    # This provides the map -> odom -> base_link transforms
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start',
            'rgb_topic': '/image_raw',
            'camera_info_topic': '/camera_info',
            'frame_id': 'base_link',
            'visual_odometry': 'true',
            'approx_sync': 'true',
            'use_sim_time': 'false',
            'qos': '1'
        }.items()
    )

    # 5. Start Nav2 (Navigation Stack)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        robot_state_publisher,
        camera_node,
        rtabmap,
        nav2
    ])
