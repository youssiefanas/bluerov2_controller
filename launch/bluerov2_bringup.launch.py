import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('bluerov2_controller')

    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='bluerov2')
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='udp://:14550@192.168.2.2:14550')
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url', default_value='udp://@127.0.0.1')
    run_init_test_arg = DeclareLaunchArgument(
        'run_initialization_test', default_value='false')

    # MAVROS
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mavros.launch.py')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
        }.items(),
    )

    # Gamepad (joy_node + teleop_twist_joy)
    gamepad_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gamepad.launch.py')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
        }.items(),
    )

    # ROV Controller
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'controller.launch.py')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'run_initialization_test': LaunchConfiguration('run_initialization_test'),
        }.items(),
    )

    # Camera Streamer
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'camera.launch.py')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
        }.items(),
    )

    return LaunchDescription([
        namespace_arg,
        fcu_url_arg,
        gcs_url_arg,
        run_init_test_arg,
        mavros_launch,
        gamepad_launch,
        controller_launch,
        camera_launch,
    ])
