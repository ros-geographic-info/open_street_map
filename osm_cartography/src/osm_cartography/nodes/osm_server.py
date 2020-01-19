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
#
# Revision $Id$

"""
Provide geographic information maps from Open Street Map on request.

This is just a toy node, useful for testing.
"""
from geographic_msgs.srv import GetGeographicMap
from geographic_msgs.srv import GetGeographicMapResponse

import rclpy
from rclpy.node import Node

from osm_cartography import xml_map


class ServerNode(Node):
    def __init__(self):
        super(ServerNode, self).__init__("osm_server")
        self.srv = self.create_service(GetGeographicMap, 'get_geographic_map', self.map_server)

        self.resp = None

    def map_server(self, req):
        """
        GetGeographicMap service callback.

        :param req: Request.
        :returns: Response.
        """
        url = str(req.url)
        self.url = url
        self.get_logger().info('[get_geographic_map] ' + url)

        # if empty URL, return existing map
        if url == '' and self.resp is not None:
            return self.resp

        self.resp = GetGeographicMapResponse()
        try:
            self.resp.map = xml_map.get_osm(url, req.bounds)
        except (IOError, ValueError) as e:
            self.get_logger().error(str(e))
            self.resp.success = False
            self.resp.status = str(e)
        else:
            self.resp.success = True
            self.resp.status = url
            self.resp.map.header.stamp = self.now()
            self.resp.map.header.frame_id = '/map'
        return self.resp


def main(args=None):
    rclpy.init(args)
    server = ServerNode()

    try:
        rclpy.spin(server)
    except rclpy.ROSInterruptException:
        pass

    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
