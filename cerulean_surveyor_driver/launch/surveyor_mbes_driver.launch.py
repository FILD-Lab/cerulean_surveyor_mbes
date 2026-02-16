from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cerulean_surveyor_driver'),
        'config',
        'surveyor_mbes_driver.yaml'
    )

    return LaunchDescription([
        Node(
            package='cerulean_surveyor_driver',
            executable='surveyor_mbes_driver',
            name='surveyor_mbes_driver',
            output='screen',
            parameters=[config]
        )
    ])
