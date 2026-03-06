import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Specify the name of the package and path to xacro file
    package_name = 'robot_description'
    file_subpath = 'urdf/robot.urdf.xacro'

    # 2. Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(package_name), file_subpath)
    
    # This command runs 'xacro' on your file to generate the robot_description string
    robot_description_raw = Command(['xacro ', xacro_file])

    # 3. Create the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': False
        }]
    )

    # 4. Launch!
    return LaunchDescription([
        node_robot_state_publisher
    ])
