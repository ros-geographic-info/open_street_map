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
Request geographic information maps from Open Street Map server.

This is just a toy node, useful for testing.
"""

import rclpy
from rclpy.node import Node

import sys

from geodesy import bounding_box

from geographic_msgs.srv import GetGeographicMap


class ClientNode(Node):
    def __init__(self, url):
        super(ClientNode, self).__init__("osm_client")

        rclpy.wait_for_service('get_geographic_map')

        try:
            get_map = rclpy.ServiceProxy('get_geographic_map', GetGeographicMap)
            resp = get_map(url, bounding_box.makeGlobal())
            if resp.success:
                print(resp.map)
            else:
                print('get_geographic_map failed, status: ', str(resp.status))

        except rclpy.ServiceException as e:
            print("Service call failed: " + str(e))


def main(args=args):
    rclpy.init(args)

    url = ''
    if len(args) == 2:
        url = args[1]
    else:
        print('usage: osm_client <map_URL>')
        sys.exit(-9)

    try:
        ClientNode(url)
    except rclpy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)
