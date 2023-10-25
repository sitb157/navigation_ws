# Copyright 2019 Open Source Robotics Foundation, Inc.
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
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    navigation_prefix = get_package_share_directory('navigation')
    # With map server
    navigation_launch_dir = LaunchConfiguration(
            'navigation_launch_dir',
            default=os.path.join(navigation_prefix, 'launch', 'navigation_launch.py'))
    navigation_params_dir = LaunchConfiguration(
            'navigation_params_dir',
            default=os.path.join(navigation_prefix, 'param', 'nav2_params.yaml'))
    cartographer_prefix = get_package_share_directory(
            'cartographer')
    cartographer_launch_dir = LaunchConfiguration(
            'cartographer_launch_dir',
            default=os.path.join(cartographer_prefix, 'launch', 'cartographer.launch.py'))
    cartographer_config_dir = LaunchConfiguration(
            'cartographer_config_dir',
            default=os.path.join(cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration(
            'configuration_basename', default='diff_drive_localization.lua')
    occupancy_grid_node_condition = LaunchConfiguration(
            'occupancy_grid_node_condition', default=True)
    rviz2_condition = LaunchConfiguration(
            'rviz2_condition', default=True)
    load_state_filename = LaunchConfiguration(
            'load_state_filename',
            default=os.path.join(cartographer_prefix, 'map', 'diff_drive.pbstream'))
    rviz_config_dir = os.path.join(navigation_prefix, 'config', 'navigation.rviz')

    declare_cartographer_config_dir = DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'
    )

    declare_configuration_basename = DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'
    )

    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
    )
     
    declare_occupancy_grid_node_condition = DeclareLaunchArgument(
            'occupancy_grid_node_condition', 
            default_value=occupancy_grid_node_condition,
            description='Run occupancygrid_node'
    )

    cartographer_localiation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch_dir),
            launch_arguments={'load_state_filename': load_state_filename,
                              'configuration_basename': configuration_basename,
                              'occupancy_grid_node_condition': occupancy_grid_node_condition,
                              'rviz2_condition': rviz2_condition}.items()
    )

    navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_dir),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': navigation_params_dir}.items()
    )
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_cartographer_config_dir,
        declare_configuration_basename,
        declare_occupancy_grid_node_condition,
        cartographer_localiation_launch,
        #navigation_launch,
        #rviz2
     ])
