import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)

def generate_launch_description():
#aaaaaaaaaaaakm
    mini_akm = GroupAction([
        launch_ros.actions.Node(
            package='robot_state_publisher', 
            node_executable='robot_state_publisher', 
            node_name='robot_state_publisher',
            output='screen',
            arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_akm_robot.urdf')],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            node_executable='static_transform_publisher', 
            node_name='base_to_laser',
            arguments=['0.125 ', '0', '0.1608','3.14', '0','0','base_footprint','laser'],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            node_executable='static_transform_publisher', 
            node_name='base_to_camera',
            arguments=['0.195', '0', '0.25','0', '0','0','base_footprint','camera_link'],),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()
    # choose your car
    ld.add_action(mini_akm)

    return ld


