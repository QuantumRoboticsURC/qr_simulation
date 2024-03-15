# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    gps_wpf_dir = get_package_share_directory("qr_simulation")
    mapviz_config_file = os.path.join(gps_wpf_dir, "gps_wpf_demo.mvc")

    # Create the launch description and populate
    ld = LaunchDescription()

    simulator = Node(
        package = 'qr_simulation',
        executable = 'simulator'
    )

    mover = Node(
        package = 'qr_simulation',
        executable = 'mover'
    )

    mapviz = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        parameters=[{"config": mapviz_config_file}]
    )

    swri = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        remappings=[
            ("fix", "gps/fix"),
        ],
    )
    
    tf2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="swri_transform",
        arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
    )

    ld.add_action(simulator)
    ld.add_action(mover)
    ld.add_action(mapviz)
    ld.add_action(swri)
    ld.add_action(tf2)

    return ld
