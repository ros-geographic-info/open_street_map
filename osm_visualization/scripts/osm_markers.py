#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Austin Robot Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Austin Robot Technology, Inc. nor the names
#    of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

"""
Create rviz markers for geographic information maps from Open Street
Map server.
"""

from __future__ import print_function

PKG_NAME = 'osm_visualization'
import roslib; roslib.load_manifest(PKG_NAME)
import rospy

import sys
import geodesy.utm

from geographic_msgs.msg import BoundingBox
from geographic_msgs.srv import GetGeographicMap
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def get_markers(geo_map):
    """Get markers for a GeographicMap.

    Returns:
        visualization markers message
    """
    msg = MarkerArray()
    yellow = ColorRGBA(r=1., g=1., b=0., a=0.8)
    forever = rospy.Duration()
    cylinder_size = Vector3(x=2., y=2., z=0.2)
    null_quaternion = Quaternion(x=0., y=0., z=0., w=1.)

    way_points = {}                     # points symbol table

    # create slightly transparent yellow disks for way-points
    index = 0
    for wp in geo_map.points:
        marker = Marker(header = geo_map.header,
                        ns = "waypoints_osm",
                        id = index,
                        type = Marker.CYLINDER,
                        action = Marker.ADD,
                        scale = cylinder_size,
                        color = yellow,
                        lifetime = forever)
        index += 1

        # convert latitude and longitude to UTM (ignoring altitude)
        pt = geodesy.utm.fromMsg(wp.position)
        #print(pt)
        marker.pose.position.x = pt.easting
        marker.pose.position.y = pt.northing
        marker.pose.orientation = null_quaternion

        way_points[wp.id.uuid] = (wp, pt)
        msg.markers.append(marker)

    #for key, val in way_points.iteritems():
    #    print(str(val[0]), str(val[1]))

    # create outline for way-point features
    green = ColorRGBA(r=0., g=1., b=0., a=0.8)
    line_width = Vector3(x=2.)
    index = 0
    for feature in geo_map.features:
        marker = Marker(header = geo_map.header,
                        ns = "features_osm",
                        id = index,
                        type = Marker.LINE_STRIP,
                        action = Marker.ADD,
                        scale = line_width,
                        color = green,
                        lifetime = forever)
        index += 1
        prev_point = None
        for mbr in feature.components:
            if mbr.uuid in way_points:
                p = Point(x = way_points[mbr.uuid][1].easting,
                          y = way_points[mbr.uuid][1].northing)
                marker.points.append(p)
                if prev_point:
                    marker.points.append(prev_point)
                prev_point = p
        if prev_point:
            marker.points.append(prev_point)
        msg.markers.append(marker)

    return msg

def markers_node(url):
    """ROS node to publish visualization markers for a GeographicMap.

    Args:
        url: uniform resource locator for get_geographic_map.
    """

    # TODO: make this topic latched
    pub = rospy.Publisher('visualization_marker_array',
                          MarkerArray, latch=True)
    msg = None # (yet)
    rospy.init_node('osm_markers')

    # first get a map from our OSM data base
    rospy.wait_for_service('get_geographic_map')
    try:
        get_map = rospy.ServiceProxy('get_geographic_map', GetGeographicMap)
        resp = get_map(url, BoundingBox())
        if resp.success:
            msg = get_markers(resp.map)
        else:
            print('get_geographic_map failed, status:', str(resp.status))

    except rospy.ServiceException, e:
        print("Service call failed:", str(e))

    if msg != None:
        # publish visualization markers (this is a latched topic)
        pub.publish(msg)
        # keep message available until shut down
        rospy.spin()

if __name__ == '__main__':

    url = 'package://osm_cartography/tests/tiny.osm'
    if len(sys.argv) == 2:
        url = sys.argv[1]
        rospy.loginfo('map URL: ' + url)

    try:
        markers_node(url)
    except rospy.ROSInterruptException: pass
