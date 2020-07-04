# Copyright 2018 Open Source Robotics Foundation, Inc.
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
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions, get_default_launch_description


def generate_launch_description():
    """
    Launch file for visualizing an OSM-derived map, and planning on top of it.
    """

    # The URL to the OSM file
    map_url = os.path.join(get_package_share_directory("osm_cartography"), "tests", "prc.osm")
    rviz_config_path = os.path.join(get_package_share_directory("osm_cartography"), "rviz", "geo_planner.rviz")

    # Start map server
    osm_server = actions.Node(
        package='osm_cartography', node_executable='osm_server', output='screen')

    # Start map visualization
    viz_osm = actions.Node(
        package='osm_cartography', node_executable='viz_osm', output='screen',
        arguments=["map_url", map_url])

    # Build a graph out of the OSM information
    route_network = actions.Node(
        package='route_network', node_executable='route_network', output='screen',
        arguments=["map_url", map_url])

    # Provide the planning service
    plan_route = actions.Node(
        package='route_network', node_executable='plan_route', output='screen')

    tf_world_map = actions.Node(
        package='tf2_ros', node_executable='static_transform_publisher', output='screen',
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "map"])

    # Use this tf to properly locate the visualization on RVIZ. 
    # The arguments must contain the correct UTM coordinates for the map used
    tf_map_local_map = actions.Node(
        package='tf2_ros', node_executable='static_transform_publisher', output='screen',
        arguments=["224401.3", "1954996.9", "0.0", "0.0", "0.0", "0.0", "map", "local_map"])

    rviz2 = actions.Node(
        package='rviz2', node_executable='rviz2', output='screen',
        arguments=["-d", rviz_config_path])

    # In RVIZ: 
    # Start Position: 2D Pose Estimate (topic: /initialpose)
    # Goal Position: 2D Nav Goal (topic: /goal)
    rviz_goal = actions.Node(
        package='route_network', node_executable='rviz_goal', output='screen')

    return LaunchDescription(
        [osm_server, viz_osm, route_network, plan_route, tf_world_map, tf_map_local_map, rviz2,
         rviz_goal])


def main():
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description())
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
