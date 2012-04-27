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
Convenience classes for manipulating OSM way-points.
"""

from __future__ import print_function

PKG = 'osm_cartography'
import roslib; roslib.load_manifest(PKG)
import rospy

import geodesy.utm

#from geographic_msgs.msg import BoundingBox
from geographic_msgs.msg import GeographicMap
from geographic_msgs.msg import WayPoint
#from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point

class WayPointUTM():
    """Representation of a map WayPoint with UTM coordinates.
    """

    def __init__(self, waypt):
        """Constructor.

        Collects relevant information from the way point message, and
        creates the corresponding geodesy.utm.UTMPoint.

        :param waypt: geographic_msgs/WayPoint message
        """
        self.way_pt = waypt
        # convert latitude and longitude to UTM (ignoring altitude)
        self.utm = geodesy.utm.fromMsg(waypt.position)

    def toPoint(self):
        """Generate geometry_msgs/Point from WayPointUTM

           :returns: corresponding geometry_msgs/Point
        """
        return self.utm.toPoint()

    def toPointXY(self):
        """Convert WayPointUTM to flattened geometry_msgs/Point.

           :returns: geometry_msgs/Point with X and Y coordinates, Z is 0.
        """
        return Point(x = self.utm.easting, y = self.utm.northing)
