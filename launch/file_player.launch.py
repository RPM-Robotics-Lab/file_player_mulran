from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node_name = LaunchConfiguration('node_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='file_player',
            description='Name of the file player node.',
        ),
        Node(
            package='file_player',
            executable='file_player',
            name=node_name,
            output='screen',
            emulate_tty=True,
        ),
    ])
