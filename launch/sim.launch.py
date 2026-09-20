# Simulation bringup: Gazebo Sim + mecanum_drive_controller (ros2_control)
# + Gazebo lidar bridged to /scan + Nav2 (SLAM by default) + RViz.
#
# Usage (from workspace root, after building + sourcing):
#   ros2 launch my_robot_nav sim.launch.py
# Headless (no GUI, e.g. over SSH):
#   ros2 launch my_robot_nav sim.launch.py gz_args:="-r -s -v 1"
# AMCL instead of SLAM (needs a saved map first):
#   ros2 launch my_robot_nav sim.launch.py slam:=False map:=/path/to/map.yaml
#
# Unlike bringup.launch.py (real hardware) this does NOT start rplidar_ros:
# /scan comes from the simulated lidar in the URDF, /odom + odom->base_link
# come from mecanum_drive_controller.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_nav')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_xacro_path = os.path.join(pkg_share, 'urdf', 'mecanum_bot_v3.urdf.xacro')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_world = os.path.join(pkg_share, 'worlds', 'nav2_world.sdf')
    default_bridge_config = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    default_controllers = os.path.join(pkg_share, 'config', 'controllers.yaml')
    default_rviz_config = os.path.join(pkg_share, 'config', 'nav2_default_view.rviz')

    xacro_path = LaunchConfiguration('xacro_path')
    params_file = LaunchConfiguration('params_file')
    world = LaunchConfiguration('world')
    gz_args = LaunchConfiguration('gz_args')
    use_slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('rviz')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw_pose = LaunchConfiguration('yaw')

    declare_xacro_path = DeclareLaunchArgument(
        'xacro_path',
        default_value=default_xacro_path,
        description='Absolute path to the robot xacro/urdf file',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Nav2 params file',
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the Gazebo world file',
    )
    declare_gz_args = DeclareLaunchArgument(
        'gz_args',
        # -r = run immediately. Headless: gz_args:="-r -s -v 1"
        default_value='-r -v 1',
        description='Extra args for gz sim (prepended to the world file)',
    )
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Run slam_toolbox instead of AMCL localization',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file (only used when slam:=False)',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock (always True in sim)',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically lifecycle-configure/activate Nav2 nodes',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Open RViz with the Nav2 view config',
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='-4.0')
    declare_z = DeclareLaunchArgument('z', default_value='0.1')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    # Gazebo Sim server (+ GUI unless -s is in gz_args)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [gz_args, ' ', world]}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mecanum_bot',
            '-x', x_pose, '-y', y_pose, '-z', z_pose, '-Y', yaw_pose,
        ],
    )

    # controller_manager lives inside the gz_ros2_control plugin, so the
    # spawners must run after the robot is spawned (chained on exit).
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_drive_controller',
            '--param-file', default_controllers,
        ],
        output='screen',
    )

    # Bridges Gazebo /scan (sim lidar) + /clock to ROS
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': default_bridge_config}],
    )

    # Twist (Nav2/teleop) -> TwistStamped (mecanum_drive_controller ~/reference)
    twist_stamper = Node(
        package='my_robot_nav',
        executable='twist_stamper.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': use_slam,
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_xacro_path,
        declare_params_file,
        declare_world,
        declare_gz_args,
        declare_slam,
        declare_map,
        declare_use_sim_time,
        declare_autostart,
        declare_rviz,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        robot_state_publisher_node,
        gz_sim,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[mecanum_drive_controller_spawner],
            )
        ),
        gz_bridge,
        twist_stamper,
        nav2_bringup_launch,
        rviz_node,
    ])
