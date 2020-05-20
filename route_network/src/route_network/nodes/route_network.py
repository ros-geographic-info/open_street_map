#!/usr/bin/env python
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

"""
Create route network messages for geographic information maps.
"""
import sys
import uuid

import geodesy.props
import geodesy.wu_point
from geodesy import bounding_box

from geographic_msgs.msg import KeyValue
from geographic_msgs.msg import RouteNetwork
from geographic_msgs.msg import RouteSegment
from geographic_msgs.srv import GetGeographicMap

import rclpy
from rclpy.node import Node

from unique_identifier_msgs.msg import UUID

PKG_NAME = 'route_network'
PKG_URL = 'http://ros.org/wiki/' + PKG_NAME


def is_oneway(feature):
    """
    One-way route predicate.
    :returns: True if feature is one way.
    """
    return geodesy.props.match(feature, {'oneway'})


def is_route(feature):
    """
    Drivable feature predicate.
    :returns: True if feature is drivable.
    """
    return geodesy.props.match(feature, {'bridge', 'highway', 'tunnel'})


def make_graph(msg):
    """
    Make RouteNetwork message.

    :param msg: GeographicMap message.
    :returns: RouteNetwork message.
    """

    uu = UUID(uuid.uuid5(uuid.NAMESPACE_URL, PKG_URL + '/map/' +
                         str(msg.id.uuid) + '/routes'))
    return RouteNetwork(header=msg.header, id=uu, bounds=msg.bounds)


def make_seg(start, end, oneway=False):
    """
    Make RouteSegment message.

    :param start:  Initial UUID.
    :param end:    Final UUID.
    :param oneway: True if segment is one-way.
    :returns: RouteSegment message.
    """
    uu = UUID(uuid.uuid5(uuid.NAMESPACE_URL, PKG_URL + '/' +
                         str(start) + '/' + str(end)))

    seg = RouteSegment(id=uu, start=start, end=end)
    if oneway:
        seg.props.append(KeyValue(key='oneway', value='yes'))
    return seg


class RouteNetNode(Node):

    def __init__(self):
        """
        ROS node to publish the route network graph for a GeographicMap.
        """
        super().__init__("route_network")
        self.config = None

        # advertise visualization marker topic
        self.pub = self.create_publisher(RouteNetwork, 'route_network', 10)
        self.graph = None

        self.get_map = self.create_client(GetGeographicMap, 'get_geographic_map')

        while not self.get_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def build_graph(self, msg):
        """
        Build RouteNetwork graph for a GeographicMap message.

        :post: self.graph = RouteNetwork message
        """
        self.map = msg
        self.map_points = geodesy.wu_point.WuPointSet(msg.points)
        self.graph = make_graph(msg)

        # process each feature marked as a route
        for feature in filter(is_route, self.map.features):
            oneway = is_oneway(feature)
            start = None
            for mbr in feature.components:
                pt = self.map_points.get(mbr.uuid)
                if pt is not None:  # known way point?
                    self.graph.points.append(pt.toWayPoint())
                    end = UUID(uuid=mbr.uuid)
                    if start is not None:
                        self.graph.segments.append(make_seg(start, end, oneway))
                        if not oneway:
                            self.graph.segments.append(make_seg(end, start))
                    start = end


def main(args=None):
    rclpy.init(args=args)
    node_class = RouteNetNode()
    try:
        rclpy.spin(node_class)
    except rclpy.exceptions.ROSInterruptException:
        pass

    node_class.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
