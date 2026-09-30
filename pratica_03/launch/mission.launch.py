import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('pratica_03')

    # Caminho do YAML de waypoints
    waypoints_yaml = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    # Inclui o launch do pose_controller
    pose_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'pose_controller.launch.py')
        )
    )

    mission_node = Node(
        package='pratica_03',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[waypoints_yaml]
    )

    return LaunchDescription([
        pose_controller_launch,
        mission_node
    ])