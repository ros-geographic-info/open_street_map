#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Jack O'Quin
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
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
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

PKG_NAME = 'osm_cartography'
import roslib; roslib.load_manifest(PKG_NAME)
import rospy

import sys
import itertools
import geodesy.utm

#import osm_cartography.way_point as way_point
from osm_cartography.geo_map import *

from geographic_msgs.msg import BoundingBox
from geographic_msgs.msg import GeoPoint
from geographic_msgs.srv import GetGeographicMap
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# dynamic parameter reconfiguration
from dynamic_reconfigure.server import Server as ReconfigureServer
import osm_cartography.cfg.VizOSMConfig as Config

class VizNode():

    def __init__(self):
        """ROS node to publish visualization markers for a GeographicMap.
        """
        rospy.init_node('viz_osm')

        # advertise visualization marker topic
        self.pub = rospy.Publisher('visualization_marker_array',
                                   MarkerArray, latch=True)
        self.msg = None
        rospy.wait_for_service('get_geographic_map')
        self.get_map = rospy.ServiceProxy('get_geographic_map',
                                          GetGeographicMap)

        # register dynamic reconfigure callback, which runs immediately
        self.reconf_server = ReconfigureServer(Config, self.reconfigure)

    def get_markers(self, msg):
        """Get markers for a GeographicMap message.

        :post: self.msg = visualization markers message
        """
        self.geo_map = GeoMap(msg)
        self.map_features = GeoMapFeatures(self.geo_map)
        self.map_points = GeoMapPoints(self.geo_map)
        self.msg = MarkerArray()
        self.mark_boundaries(ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8))
        self.mark_way_points(ColorRGBA(r=1., g=1., b=0., a=0.8))
        self.mark_features(lambda(f): match_tags(f, road_tags),
                           ColorRGBA(r=8., g=0.2, b=0.2, a=0.8),
                           "roads_osm")
        self.mark_features(lambda(f): match_tags(f, {'building'}),
                           ColorRGBA(r=0., g=0.3, b=0.7, a=0.8),
                           "buildings_osm")
        self.mark_features(lambda(f): match_tags(f, {'railway'}),
                           ColorRGBA(r=0., g=0.7, b=.7, a=0.8),
                           "railroad_osm")
        self.mark_features(lambda(f): match_tags(f, {'amenity', 'landuse'}),
                           ColorRGBA(r=0., g=1., b=0., a=0.5),
                           "other_osm")

    def mark_boundaries(self, color):
        # draw outline of map boundaries
        marker = Marker(header = self.geo_map.header(),
                        ns = "bounds_osm",
                        id = 0,
                        type = Marker.LINE_STRIP,
                        action = Marker.ADD,
                        scale = Vector3(x=2.),
                        color = color,
                        lifetime = rospy.Duration())
    
        # Convert bounds latitudes and longitudes to UTM (no
        # altitude), convert UTM points to geometry_msgs/Point
        # :todo: invent a better map bounds interface
        bounds = self.geo_map.bounds()
        p0 = geodesy.utm.fromLatLong(bounds.min_latitude,
                                     bounds.min_longitude).toPoint()
        p1 = geodesy.utm.fromLatLong(bounds.min_latitude,
                                     bounds.max_longitude).toPoint()
        p2 = geodesy.utm.fromLatLong(bounds.max_latitude,
                                     bounds.max_longitude).toPoint()
        p3 = geodesy.utm.fromLatLong(bounds.max_latitude,
                                     bounds.min_longitude).toPoint()
    
        # add line strips to bounds marker
        marker.points.append(p0)
        marker.points.append(p1)
        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p2)
        marker.points.append(p3)
        marker.points.append(p3)
        marker.points.append(p0)
        self.msg.markers.append(marker)

    def mark_features(self, predicate, color, namespace):
        """Create outline for map features

        :param predicate: function to match desired features
        :param color: RGBA value
        :param namespace: Rviz namespace.

        :todo: differentiate tags for: highway, building, bridge,
               tunnel, amenity, etc.
        """
        index = 0
        for feature in itertools.ifilter(predicate,
                                         self.map_features):
            marker = Marker(header = self.geo_map.header(),
                            ns = namespace,
                            id = index,
                            type = Marker.LINE_STRIP,
                            action = Marker.ADD,
                            scale = Vector3(x=2.),
                            color = color,
                            lifetime = rospy.Duration())
            index += 1
            prev_point = None
            for mbr in feature.components:
                wu_point = self.map_points.get(mbr.uuid)
                if wu_point:    # this component is a way point
                    p = wu_point.toPointXY()
                    if prev_point:
                        marker.points.append(prev_point)
                        marker.points.append(p)
                    prev_point = p
            self.msg.markers.append(marker)

    def mark_way_points(self, color):
        """Create slightly transparent disks for way-points.

        :param color: disk RGBA value
        """
        cylinder_size = Vector3(x=2., y=2., z=0.2)
        null_quaternion = Quaternion(x=0., y=0., z=0., w=1.)
        index = 0
        for wp in self.map_points:
            marker = Marker(header = self.geo_map.header(),
                            ns = "waypoints_osm",
                            id = index,
                            type = Marker.CYLINDER,
                            action = Marker.ADD,
                            scale = cylinder_size,
                            color = color,
                            lifetime = rospy.Duration())
            index += 1
            # use easting and northing coordinates (ignoring altitude)
            marker.pose.position = wp.toPointXY()
            marker.pose.orientation = null_quaternion
            self.msg.markers.append(marker)
    
    def reconfigure(self, config, level):
        """Dynamic reconfigure callback.

        :param config: New configuration.
        :param level:  0x00000001 bit set if URL changed (ignored).

        :returns: New config if valid, old one otherwise. That updates
                  the dynamic reconfigure GUI window.
        """
        # treat an empty URL as a valid "do nothing" command
        if config.map_url == '':
            self.config = config        # save new config
            return config

        rospy.loginfo('Map URL: ' + str(config.map_url))

        try:
            resp = self.get_map(config.map_url, BoundingBox())
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed:", str(e))
            # ignore new config, it failed
        else:                           # get_map returned
            if resp.success:
                self.get_markers(resp.map)
                self.config = config    # save new URL
                # publish visualization markers (on a latched topic)
                self.pub.publish(self.msg)
            else:
                print('get_geographic_map failed, status:', str(resp.status))

        return self.config
    
def main():
    viznode = VizNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())
