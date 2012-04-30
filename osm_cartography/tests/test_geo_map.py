#!/usr/bin/env python

PKG='osm_cartography'
import roslib; roslib.load_manifest(PKG)

import unittest

from geographic_msgs.msg import GeographicMap
#from geographic_msgs.msg import GeoPoint
#from geographic_msgs.msg import WayPoint
#from geometry_msgs.msg import Point

from osm_cartography.geo_map import *
import osm_cartography.xml_map

#def fromLatLong(lat, lon, alt=float('nan')):
#    """Generate WayPoint from latitude, longitude and (optional) altitude.
#
#    :returns: minimal WayPoint object just for test cases.
#    """
#    geo_pt = GeoPoint(latitude = lat, longitude = lon, altitude = alt)
#    return WayPoint(position = geo_pt)

class TestGeoMap(unittest.TestCase):
    """Unit tests for GeoMap class.
    """

    def test_empty_map(self):
        gm = GeoMap(GeographicMap())
        self.assertEqual(gm.n_points, 0)
        self.assertEqual(gm.n_features, 0)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_geo_map_py', TestGeoMap) 
