#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    imu_node = Node(package='imu_get_cpp',
                    executable='publisher_imu_node',
                    name='publisher_imu_node',		
                    output='screen',                              
                    )
    return LaunchDescription([
        imu_node,
    ])
