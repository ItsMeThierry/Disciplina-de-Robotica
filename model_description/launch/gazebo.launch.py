import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Aponta para o arquivo SDF do seu robô vascaíno
    model_path = os.path.expanduser('~/.gazebo/models/my_robot/model.sdf')
    
    return LaunchDescription([
        # Inicia o servidor e o cliente do Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Nó do ROS2 para injetar o robô no Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'vasco_robot', '-file', model_path, '-x', '0', '-y', '0', '-z', '0.2'],
            output='screen'
        )
    ])