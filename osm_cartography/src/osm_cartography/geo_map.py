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

import itertools

import geodesy.utm
import geodesy.wu_point

from geographic_msgs.msg import GeographicMap
from geographic_msgs.msg import WayPoint
from geometry_msgs.msg import Point

class GeoMap():
    """
    :class:`GeoMap` provides an internal GeographicMap representation.
    """

    def __init__(self, gmap):
        """Constructor.

        Collects relevant information from the geographic map message,
        and provides convenient access to the data.

        :param gmap: geographic_msgs/GeographicMap message
        """
        self.gmap = gmap

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
