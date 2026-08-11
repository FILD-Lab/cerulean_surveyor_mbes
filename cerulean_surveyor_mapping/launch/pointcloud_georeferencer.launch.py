from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_frame', default_value='map'),
        Node(
            package='cerulean_surveyor_mapping',
            executable='pointcloud_georeferencer_node',
            name='pointcloud_georeferencer_node',
            output='screen',
            parameters=[{'map_frame': LaunchConfiguration('map_frame')}],
        )
    ])
