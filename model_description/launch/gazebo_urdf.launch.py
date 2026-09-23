import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Caminho direto para o ficheiro URDF que acabámos de criar
    urdf_file = os.path.expanduser('~/ros2_ws/src/model_description/urdf/vasco_robot.urdf')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'vasco_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.2'],
            output='screen'
        )
    ])