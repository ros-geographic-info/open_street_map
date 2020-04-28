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


OSM_CARTOGRAPHY_SHARE_DIR = get_package_share_directory('osm_cartography')


def generate_launch_description():
    """
    Test launch file for visualizing OSM data

    This is NOT for general use. It defines a static transform from
    the /map frame to /local_map, which is specific to the Pickle
    Research Campus of the University of Texas at Austin.

    To visualize your own maps, define a nearby /local_map
    frame for the relevant area.
    """
    map_url = "package://osm_cartography/tests/prc.osm"

    # Transform /map into /local_map frame. Rviz cannot handle large
    # UTM coordinates, they require double precision, so we visualize
    # using the /local_map.  Start the transform early to minimize TF
    # errors in rviz.
    local_map_tf = actions.Node(
        package='tf2_ros', node_executable='static_transform_publisher', output='screen',
        arguments=["622150", "3362350", "0.0", "0.0", "0.0", "0.0", "map", "local_map"])


    # launch the map visualization nodes
    # include launch file
    viz_osm_ld = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(OSM_CARTOGRAPHY_SHARE_DIR, '/viz_osm.launch.py')),
            launch_arguments={'url': map_url}.items(),
        )

    
    #<include file="$(find osm_cartography)/launch/viz_osm.launch" >
    #<arg name="url" value="$(arg url)" />
    #</include>
    return LaunchDescription([local_map_tf, viz_osm_ld])


def main(argv):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description(prefix_output_with_name=False))
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main(sys.argv)
