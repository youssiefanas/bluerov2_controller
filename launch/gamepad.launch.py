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
    joy_device_arg = DeclareLaunchArgument(
        'joy_device', default_value='/dev/input/js0')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'device_id': 0,
            'deadzone': 0.2,
            'autorepeat_rate': 0.0,
        }],
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            os.path.join(pkg_share, 'config', 'xbox_teleop.yaml'),
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
        ],
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        joy_device_arg,
        joy_node,
        teleop_node,
    ])
