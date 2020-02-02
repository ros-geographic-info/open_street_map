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

import sys

from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch.actions import DeclareLaunchArgument
from launch_ros import actions, get_default_launch_description


def generate_launch_description():
    """
    Launch file for visualizing OSM route networks
    arg: url = Uniform Resource Locator for map data
    """
    url = ""
    # Network graph node
    route_network = actions.Node(
        package='route_network', node_executable='route_network', output='screen',
        arguments=["map_url", url])

    # Route plan visualization
    viz_routes = actions.Node(
        package='route_network', node_executable='viz_routes', output='screen',
        arguments=["map_url", url])

    return LaunchDescription(
        [DeclareLaunchArgument('url', default_value='', description="Map url"), route_network, viz_routes])


def main():
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description())
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
