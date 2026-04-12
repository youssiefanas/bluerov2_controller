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

    camera_node = Node(
        package='bluerov2_controller',
        executable='camera_streamer',
        name='camera_streamer',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            os.path.join(pkg_share, 'config', 'camera_params.yaml'),
        ],
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        camera_node,
    ])
