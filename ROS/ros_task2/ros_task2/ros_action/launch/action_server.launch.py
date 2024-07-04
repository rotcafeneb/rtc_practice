import os

from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_path('ros_action')
    config_path = os.path.join(pkg_share, 'config', 'action_server.yaml')

    config_arg = DeclareLaunchArgument(name='config', default_value=config_path,
                                       description='Absolute path to config file')

    action_server = Node(
        package='ros_action',
        executable='action_server',
        name='action_server',
        output='screen',
        emulate_tty=False,
        parameters=[LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        action_server
    ])