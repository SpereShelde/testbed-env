import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the launch directory
    ekf_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'ekf.yaml')
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    urdf = os.path.join(get_package_share_directory('wheeltec_robot_urdf'),'urdf','mini_akm_robot.urdf')
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
    )
    velocity_smoother = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'velocity_smoother-launch.py')),
    )
    #choose your car,the default car is mini_mec 
    choose_car = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
    )
    base_to_link = launch_ros.actions.Node(
            package='tf2_ros', 
            node_executable='static_transform_publisher', 
            node_name='base_to_link',
            arguments=['0', '0', '0','0', '0','0','base_footprint','base_link'],
    )
    base_to_gyro = launch_ros.actions.Node(
            package='tf2_ros', 
            node_executable='static_transform_publisher', 
            node_name='base_to_gyro',
            arguments=['0', '0', '0','0', '0','0','base_footprint','gyro_link'],
    )
    robot_ekf = launch_ros.actions.Node(
            package='robot_localization',
            node_executable='ekf_node',
            parameters=[ekf_config],
            remappings=[("odometry/filtered", "odom_combined")]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher', 
            node_executable='joint_state_publisher', 
            node_name='joint_state_publisher',
    )
    robot_state_publisher = launch_ros.actions.Node(
            package='robot_state_publisher', 
            node_executable='robot_state_publisher', 
            node_name='robot_state_publisher',
            output='screen',
            arguments=[urdf],)

    ld = LaunchDescription()
    ld.add_action(wheeltec_robot)
#     ld.add_action(base_to_link)
#     ld.add_action(base_to_gyro)
#     ld.add_action(joint_state_publisher_node)
    #ld.add_action(robot_state_publisher)
#     ld.add_action(choose_car)
#     ld.add_action(robot_ekf)

    return ld


