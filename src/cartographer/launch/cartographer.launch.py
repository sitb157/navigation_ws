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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cartographer_prefix = get_package_share_directory(
        'cartographer')
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='diff_drive.lua')
    load_state_filename = LaunchConfiguration(
        'load_state_filename', default='')
    occupancy_grid_node_condition = LaunchConfiguration(
        'occupancy_grid_node_condition', default='true')
    rviz2_condition = LaunchConfiguration(
        'rviz2_condition', default='true')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    rviz_config_dir = os.path.join(cartographer_prefix, 'config', 'cartographer.rviz')

    remappings=[('scan', 'diff_drive/scan'),
                ('imu', 'diff_drive/imu'),
                ('odom', 'diff_drive/odometry')]

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
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
    )
     
    declare_resolution = DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'
    )

    declare_publish_period_sec = DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'
    )

    declare_occupancy_grid_node_condition = DeclareLaunchArgument(
            'occupancy_grid_node_condition', 
            default_value=occupancy_grid_node_condition,
            description='Run occupancygrid_node'
    )

    declare_rviz2_condition = DeclareLaunchArgument(
            'rviz2_condition', 
            default_value=rviz2_condition,
            description='Run rviz2'
    )

    cartographer_node =  Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters = [{'use_sim_time': use_sim_time}],
            remappings=remappings,
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '-load_state_filename', load_state_filename]
    )

    cartographer_occupancy_grid_node = Node(
            package = 'cartographer_ros',
            executable = 'cartographer_occupancy_grid_node',
            parameters = [{'use_sim_time': use_sim_time},
                          {'resolution': 0.05}],
            condition=IfCondition(LaunchConfiguration('occupancy_grid_node_condition'))
    )

    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz2_condition')),
            output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_cartographer_config_dir,
        declare_configuration_basename,
        declare_resolution,
        declare_publish_period_sec,
        declare_occupancy_grid_node_condition,
        declare_rviz2_condition,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz2
     ])
