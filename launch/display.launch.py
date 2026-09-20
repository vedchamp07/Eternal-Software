import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_nav')
    default_xacro_path = os.path.join(pkg_share, 'urdf', 'mecanum_bot_v3.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'config', 'urdf_view.rviz')

    xacro_path = LaunchConfiguration('xacro_path')

    declare_xacro_path = DeclareLaunchArgument(
        'xacro_path',
        default_value=default_xacro_path,
        description='Absolute path to the robot xacro/urdf file',
    )

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        declare_xacro_path,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
