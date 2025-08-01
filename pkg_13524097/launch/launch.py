#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('pkg_13524097'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package = 'pkg_13524097',
            executable = 'node.py',
            name = 'node',
            parameters = [config_file],
            output = 'screen'
        )
    ])