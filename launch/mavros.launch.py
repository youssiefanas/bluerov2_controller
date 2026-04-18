from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='bluerov2')
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='udp://192.168.2.1:14550@192.168.2.2')
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url', default_value='udp://@127.0.0.1')
    # log_level_arg = DeclareLaunchArgument(
        # 'log_level', default_value='info')

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mavros'), 'launch', 'node.launch'
            ])
        ),
        launch_arguments={
            'pluginlists_yaml': PathJoinSubstitution([
                FindPackageShare('mavros'), 'launch', 'apm_pluginlists.yaml'
            ]),
            'config_yaml': PathJoinSubstitution([
                FindPackageShare('mavros'), 'launch', 'apm_config.yaml'
            ]),
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': '1',
            'tgt_component': '1',
            'log_output': 'screen',
            'fcu_protocol': 'v2.0',
            'respawn_mavros': 'false',
            'namespace': LaunchConfiguration('namespace'),
        }.items(),
    )

    return LaunchDescription([
        namespace_arg,
        fcu_url_arg,
        gcs_url_arg,
        mavros_launch,
    ])
