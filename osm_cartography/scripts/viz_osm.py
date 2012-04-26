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
import geodesy.utm

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

    def get_markers(self, geo_map):
        """Get markers for a GeographicMap.

        :returns: visualization markers message
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
    
        # create outline for map features
        #
        # :todo: differentiate tags for: highway, building, bridge,
        #        tunnel, amenity, etc.
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
                    if prev_point:
                        marker.points.append(prev_point)
                        marker.points.append(p)
                    prev_point = p
            msg.markers.append(marker)
    
        # draw outline of map boundaries
        red = ColorRGBA(r=1., g=0., b=0., a=0.8)
        marker = Marker(header = geo_map.header,
                        ns = "bounds_osm",
                        id = 0,
                        type = Marker.LINE_STRIP,
                        action = Marker.ADD,
                        scale = line_width,
                        color = red,
                        lifetime = forever)
    
        # convert latitudes and longitudes to UTM (no altitude)
        utm0 = geodesy.utm.fromLatLong(geo_map.bounds.min_latitude,
                                       geo_map.bounds.min_longitude)
        utm1 = geodesy.utm.fromLatLong(geo_map.bounds.min_latitude,
                                       geo_map.bounds.max_longitude)
        utm2 = geodesy.utm.fromLatLong(geo_map.bounds.max_latitude,
                                       geo_map.bounds.max_longitude)
        utm3 = geodesy.utm.fromLatLong(geo_map.bounds.max_latitude,
                                       geo_map.bounds.min_longitude)
    
        # convert UTM points to geometry_msgs/Point
        p0 = utm0.toPoint()
        p1 = utm1.toPoint()
        p2 = utm2.toPoint()
        p3 = utm3.toPoint()
    
        # add line strips to bounds marker
        marker.points.append(p0)
        marker.points.append(p1)
        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p2)
        marker.points.append(p3)
        marker.points.append(p3)
        marker.points.append(p0)
        #print(marker)
        msg.markers.append(marker)
    
        return msg
    
    def reconfigure(self, config, level):
        """Dynamic reconfigure callback.

        :param config: New configuration.
        :param level:  0x00000001 if URL changed (currently ignored).

        :returns: New config if valid, old one otherwise. That updates
                  the dynamic reconfigure GUI window.
        """
        # treat an empty URL as a valid, "do nothing" command
        if config.map_url == '':
            return config

        rospy.loginfo('Map URL: ' + str(config.map_url))

        try:
            resp = self.get_map(config.map_url, BoundingBox())
            if resp.success:
                self.msg = self.get_markers(resp.map)
                self.config = config    # save new URL
                # publish visualization markers (on a latched topic)
                self.pub.publish(self.msg)
            else:
                print('get_geographic_map failed, status:', str(resp.status))

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed:", str(e))

        return self.config
    
def main():
    viznode = VizNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())
