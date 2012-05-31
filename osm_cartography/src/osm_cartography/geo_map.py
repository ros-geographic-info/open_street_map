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

.. warning::

   This module is *unstable* and still evolving. It will probably end
   up residing in some other package.

.. _`geographic_msgs/BoundingBox`: http://ros.org/doc/api/geographic_msgs/html/msg/BoundingBox.html
.. _`geographic_msgs/GeographicMap`: http://ros.org/doc/api/geographic_msgs/html/msg/GeographicMap.html
.. _`std_msgs/Header`: http://ros.org/doc/api/std_msgs/html/msg/Header.html

"""

PKG = 'osm_cartography'
import roslib; roslib.load_manifest(PKG)

from geographic_msgs.msg import GeographicMap
from geographic_msgs.msg import WayPoint
from geometry_msgs.msg import Point

class GeoMap():
    """
    :class:`GeoMap` provides an internal
    `geographic_msgs/GeographicMap`_ representation.

    :param gmap: `geographic_msgs/GeographicMap`_ message.
    """

    def __init__(self, gmap):
        """Constructor.

        Collects relevant information from the geographic map message,
        and provides convenient access to the data.
        """
        self.gmap = gmap

        # Initialize feature information.
        self.feature_ids = {}           # feature symbol table
        self.n_features = len(self.gmap.features)
        for fid in xrange(self.n_features):
            feat = self.gmap.features
            self.feature_ids[feat[fid].id.uuid] = fid

    def bounds(self):
        """
        :returns: `geographic_msgs/BoundingBox`_ from the `geographic_msgs/GeographicMap`_ message.
        """
        return self.gmap.bounds

    def header(self):
        """
        :returns: `std_msgs/Header`_ from the `geographic_msgs/GeographicMap`_ message. 
        """
        return self.gmap.header

class GeoMapFeatures():
    """
    :class:`GeoMapFeatures` provides a filtering iterator for the
    features in a :class:`osm_cartography.geo_map.GeoMap`.

    :param geomap: :class:`GeoMap` object.

    :class:`GeoMapFeatures` provides these standard container operations:

    .. describe:: len(features)

       :returns: The number of points in the set.

    .. describe:: features[uuid]
 
       :returns: The point with key *uuid*.  Raises a :exc:`KeyError`
                 if *uuid* is not in the set.
 
    .. describe:: uuid in features
 
       :returns: ``True`` if *features* has a key *uuid*, else ``False``.
 
    .. describe:: uuid not in features
 
       Equivalent to ``not uuid in features``.
 
    .. describe:: iter(features)
 
       :returns: An iterator over all the features.  This is a
                 shortcut for :meth:`iterkeys`.

       Example::

           gm = GeoMap(msg)
           gf = GeoMapFeatures(gm)
           for feat in gf:
               print str(feat)
    """

    def __init__(self, geomap):
        """Constructor. """
        self.gmap = geomap

    def __contains__(self, item):
        """ Feature set membership. """
        return item in self.gmap.feature_ids

    def __getitem__(self, key):
        """ Feature accessor."""
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

        :returns: :class:`geodesy.wu_point.WuPoint` object for next point
        :raises: :exc:`StopIteration` when finished.
        """
        i = self.iter_index
        if i >= self.gmap.n_features:
            raise StopIteration
        self.iter_index = i + 1
        return self.gmap.gmap.features[i]
