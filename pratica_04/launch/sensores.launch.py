import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('pratica_04')
    urdf_path = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'vasco_robot.urdf')
    pid_config = os.path.join(pkg_share, 'config', 'trajectory_params.yaml')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value= urdf_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': True
        }]
    )

    gazebo = ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
         '-s', 'libgazebo_ros_factory.so'],
        output='screen',
        # SIGINT tem 15s para encerrar, depois SIGTERM, depois SIGKILL
        # rolou problema comigo :)
        sigterm_timeout='15.0',
        sigkill_timeout='10.0',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'vasco_robot', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    load_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    trajectory_controller_node = Node(
        package='pratica_04',
        executable='trajectory_controller_node',
        name='trajectory_controller_node',
        output='screen',
        parameters=[
            pid_config,
            {'use_sim_time': True}
        ],
        remappings=[
            ('/odom',    '/diff_cont/odom'),
            ('/cmd_vel', '/diff_cont/cmd_vel_unstamped'),
        ]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster, load_diff_drive_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_diff_drive_controller,
                on_exit=[trajectory_controller_node],
            )
        ),
    ])