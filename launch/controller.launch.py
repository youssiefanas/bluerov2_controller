import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bluerov2_controller')

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='bluerov2')
    run_init_test_arg = DeclareLaunchArgument(
        'run_initialization_test', default_value='false')

    controller_node = Node(
        package='bluerov2_controller',
        executable='rov_controller',
        name='rov_controller',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            os.path.join(pkg_share, 'config', 'controller_params.yaml'),
            {'run_initialization_test': LaunchConfiguration('run_initialization_test')},
        ],
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        run_init_test_arg,
        controller_node,
    ])
