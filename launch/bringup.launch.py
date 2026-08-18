import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_xacro_path = os.path.join(pkg_share, 'urdf', 'mecanum_bot_v3.urdf.xacro')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    xacro_path = LaunchConfiguration('xacro_path')
    params_file = LaunchConfiguration('params_file')
    use_slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

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

    declare_slam = DeclareLaunchArgument(
        'slam',
        # No saved map exists yet (maps/ is empty), so default to SLAM mode
        # (slam_toolbox builds the map live). Once a map is saved, launch
        # with slam:=False map:=<path_to_map.yaml> for AMCL localization instead.
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
        default_value='False',
        description='Use simulation clock (leave False on real hardware)',
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically lifecycle-configure/activate Nav2 nodes',
    )

    robot_description = Command(['xacro ', xacro_path])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    # RPLIDAR A1, wired via USB-serial (CP210x) directly to the onboard computer.
    # NOTE: this RPi has shown undervoltage + USB timeout errors powering the
    # lidar directly off its own USB port - route it through a powered hub
    # before trusting this for real runs.
    # NOTE: serial_port is hardcoded to /dev/ttyUSB1, confirmed by testing
    # (S/N 9754FA89C7E19EC8BCE499F01C835670). /dev/ttyUSB0 is a different
    # device (likely the ESP32, same generic CP2102 chip). USB enumeration
    # order isn't guaranteed across reboots - if the lidar stops responding,
    # check which ttyUSBx it actually landed on before assuming it's broken.
    # rplidar_ros's own launch file hardcodes /dev/ttyUSB0 with no declared
    # launch arguments, so the node is defined directly here instead of
    # included, to actually override the port.
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    # NOTE: no odometry source is included here yet. odom -> base_link TF and
    # /odom must come from the ESP32 motor controller firmware (still in
    # progress) before this stack can actually navigate, not just plan on paper.
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

    return LaunchDescription([
        declare_xacro_path,
        declare_params_file,
        declare_slam,
        declare_map,
        declare_use_sim_time,
        declare_autostart,
        robot_state_publisher_node,
        rplidar_node,
        nav2_bringup_launch,
    ])
