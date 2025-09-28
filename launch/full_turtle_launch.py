#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim in a new terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'turtlesim', 'turtlesim_node'],
            output='screen'
        ),
        
        # Launch TurtleCommander node in a new terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'arl_ros2_project', 'turtle_commander'],
            output='screen'
        ),
        
        # Launch shape_node in a new terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'arl_ros2_project', 'shape_node'],
            output='screen'
        ),
    ])
