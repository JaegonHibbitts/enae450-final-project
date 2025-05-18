#!/usr/bin/env python3

import os
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution

def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='2.75') #Nessesary for start position
    y_pose = LaunchConfiguration('y_pose', default='1.5')  #Nessesary for start position
    world_name = LaunchConfiguration('world', default='maze_2.world')

    # World file path using substitutions
    world_path = PathJoinSubstitution([
        FindPackageShare('interfacing'),
        'worlds',
        world_name
    ])

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            )
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            )
        )
    )

    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn TurtleBot3
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'spawn_turtlebot3.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': '3.14159265358979323846'  # 180 degrees in radians
        }.items()
    )

    
    # spawn_entity_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'turtlebot3',
    #         '-topic', 'robot_description',
    #         '-x', x_pose,
    #         '-y', y_pose,
    #         '-z', '0.0',
    #         '-Y', '3.14159'  # Yaw in radians = 180 degrees
    #     ],
    #     output='screen'
    # )

    # delayed_spawn = TimerAction(
    #     period=3.0,
    #     actions=[spawn_entity_cmd]
    # )

    # Your custom Gazebo_h1 node
    gazebo_h1_node = Node(
        package='interfacing',
        executable='Gazebo',
        output='screen'
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        #delayed_spawn,
        spawn_turtlebot_cmd,
        gazebo_h1_node
    ])