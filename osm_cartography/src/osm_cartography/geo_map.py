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
.. module:: geo_map

Class for manipulating GeographicMap data.

"""

from __future__ import print_function

PKG = 'osm_cartography'
import roslib; roslib.load_manifest(PKG)
import rospy

import geodesy.utm
import osm_cartography.way_point

from geographic_msgs.msg import GeographicMap
from geographic_msgs.msg import WayPoint
from geometry_msgs.msg import Point

class GeoMap():
    """
    :class:`GeoMap` represents a map WayPoint with UTM coordinates.
    """

    def __init__(self, gmap):
        """Constructor.

        Collects relevant information from the geographic map message,
        and provides convenient access to the data.

        :param gmap: geographic_msgs/GeographicMap message
        """
        self.gmap = gmap

        # Initialize way point information.
        self.way_point_ids = {}         # points symbol table
        self.n_points = len(self.gmap.points)
        for wid in xrange(self.n_points):
            wp = self.gmap.points
            self.way_point_ids[wp[wid].id.uuid] = wid

        # Create empty list of UTM points, corresponding to map points.
        # They will be evaluated lazily, when first needed.
        self.utm_points = [None for wid in xrange(self.n_points)]

        # Initialize feature information.
        self.feature_ids = {}           # feature symbol table
        self.n_features = len(self.gmap.features)
        for fid in xrange(self.n_features):
            feat = self.gmap.features
            self.feature_ids[feat[fid].id.uuid] = fid

    def bounds(self):
        """ Get GeographicMap message BoundingBox
        """
        return self.gmap.bounds

    def header(self):
        """ Get GeographicMap message Header
        """
        return self.gmap.header

    def get_point_with_utm(self, index):
        """ Get way point with UTM coordinates.

        :param index: Index of point in self.
        :returns: Corresponding WuPoint object.
        """
        way_pt = self.gmap.points[index]
        utm_pt = self.utm_points[index]
        if not utm_pt:
            utm_pt = geodesy.utm.fromMsg(way_pt.position)
            self.utm_points[index] = utm_pt
        return osm_cartography.way_point.WuPoint(way_pt, utm=utm_pt)

class GeoMapFeatures():
    """
    :class:`GeoMapFeatures` provides an filtering iterator for the
    features in a GeoMap.
    """

    def __init__(self, gmap):
        """Constructor.

        Collects relevant feature information from the geographic map
        message, and provides convenient access to the data.

        :param gmap: geographic_msgs/GeographicMap message
        """
        self.gmap = gmap

    def __contains__(self, item):
        """ Feature set membership. """
        return item in self.gmap.feature_ids

    def __getitem__(self, key):
        """ Feature accessor.
    
        :returns: Matching Feature.
        :raises: :exc:`KeyError` if no such feature
        """
        index = self.gmap.feature_ids[key]
        return self.gmap.features[index]

    def __iter__(self):
        """ Features iterator. """
        self.iter_index = 0
        return self

    def __len__(self):
        """Features vector length."""
        return len(self.gmap.gmap.features)

    def next(self):
        """ Next matching feature.

        :returns: WuPoint object for next point
        :raises: :exc:`StopIteration` when finished.
        """
        i = self.iter_index
        if i >= self.gmap.n_features:
            raise StopIteration
        self.iter_index = i + 1
        return self.gmap.gmap.features[i]

class GeoMapPoints():
    """
    :class:`GeoMapPoints` provides an iterator for the way points in a
    GeoMap.
    """

    def __init__(self, gmap):
        """Constructor.

        Collects relevant way point information from the geographic
        map message, and provides convenient access to the data.

        :param gmap: geographic_msgs/GeographicMap message
        """
        self.gmap = gmap

    def __contains__(self, item):
        """ Points set membership. """
        return item in self.gmap.way_point_ids

    def __getitem__(self, key):
        """ Points accessor.

        :returns: WuPoint object for matching point
        :raises: :exc:`KeyError` if no such point
        """
        index = self.gmap.way_point_ids[key]
        return self.gmap.get_point_with_utm(index)

    def __iter__(self):
        """ Points iterator. """
        self.iter_index = 0
        return self

    def __len__(self):
        """Points vector length."""
        return len(self.gmap.gmap.points)

    def next(self):
        """ Next matching point.

        :returns: WuPoint object for next point
        :raises: :exc:`StopIteration` when finished.
        """
        i = self.iter_index
        if i >= self.gmap.n_points:
            raise StopIteration
        self.iter_index = i + 1
        return self.gmap.get_point_with_utm(i)

    #def toPoint(self):
    #    """Generate geometry_msgs/Point from GeoMap
    #
    #       :returns: corresponding geometry_msgs/Point
    #    """
    #    return self.utm.toPoint()
    #
    #def toPointXY(self):
    #    """Generate flattened geometry_msgs/Point from GeoMap.
    #
    #       :returns: geometry_msgs/Point with X and Y coordinates, Z is 0.
    #    """
    #    return Point(x = self.utm.easting, y = self.utm.northing)
