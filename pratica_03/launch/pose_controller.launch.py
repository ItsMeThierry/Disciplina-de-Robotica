from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pratica_03')
    params = os.path.join(pkg_share, 'config', 'pose_controller.yaml')

    return LaunchDescription([
        Node(
            package='pratica_03',
            executable='pose_controller',
            name='pose_controller',
            output='screen',
            parameters=[params],
        ),
    ])