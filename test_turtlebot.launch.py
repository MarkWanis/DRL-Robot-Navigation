#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_td3 = get_package_share_directory('td3')
    rviz_file = os.path.join(pkg_td3, 'launch', 'pioneer3dx.rviz')

    return LaunchDescription([
        # Your RL/test script (runs on real robot)
        Node(
            package='td3',
            executable='test_velodyne_node.py',
            output='screen'
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen'
        ),
    ])
