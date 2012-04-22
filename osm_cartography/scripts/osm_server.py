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
Provide geographic information maps from Open Street Map on request.

This is just a toy node, useful for testing.
"""

from __future__ import print_function

PKG_NAME = 'osm_cartography'
import roslib; roslib.load_manifest(PKG_NAME)
import rospy

from geographic_msgs.srv import GetGeographicMap
from geographic_msgs.srv import GetGeographicMapResponse

import osm_map

def map_server(req):

    url = req.url
    rospy.loginfo('get_geographic_map: ' + str(url))

    # parse the URL
    filename = ''
    if url.startswith('file:///'):
        filename = url[7:]
        print(filename)
    elif url.startswith('package://'):
        pkg_name, slash, pkg_path = url[10:].partition('/')
        pkg_dir = roslib.packages.get_pkg_dir(pkg_name)
        filename = pkg_dir + '/' + pkg_path

    #print(filename)
    f = open(filename, 'r')
    parser = osm_map.ParseOSM()

    resp = GetGeographicMapResponse()
    resp.success = True
    resp.status = filename
    resp.map = parser.get_map(f)
    resp.map.header.stamp = rospy.Time.now()
    resp.map.header.frame_id = '/map'

    return(resp)

def server_node():
    srv = rospy.Service('get_geographic_map', GetGeographicMap, map_server)
    rospy.init_node('osm_server')
    rospy.spin()

if __name__ == '__main__':
    try:
        server_node()
    except rospy.ROSInterruptException: pass
