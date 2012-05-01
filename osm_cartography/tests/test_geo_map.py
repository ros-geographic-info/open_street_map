#!/usr/bin/env python

PKG='osm_cartography'
import roslib; roslib.load_manifest(PKG)

import unittest

from geographic_msgs.msg import BoundingBox
from geographic_msgs.msg import GeographicMap
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

    def test_tiny_map(self):
        parser = osm_cartography.xml_map.ParseOSM()
        gm = GeoMap(parser.get_map('package://osm_cartography/tests/tiny.osm',
                                   BoundingBox()))
        # :todo: deeper results verification
        self.assertEqual(gm.n_points, 3)
        self.assertEqual(gm.n_features, 2)

    def test_prc_map(self):
        parser = osm_cartography.xml_map.ParseOSM()
        gm = GeoMap(parser.get_map('package://osm_cartography/tests/prc.osm',
                                   BoundingBox()))
        # :todo: deeper results verification
        self.assertEqual(gm.n_points, 986)
        self.assertEqual(gm.n_features, 84)

        gpts = GeoMapPoints(gm)
        uu = '8e0b7d8a-c433-5c42-be2e-fbd97ddff9ac'
        self.assertTrue(uu in gpts)
        wpt, upt = gpts[uu]
        self.assertEqual(wpt.id.uuid, uu)
        self.assertAlmostEqual(wpt.position.latitude, 30.370945, places=3)
        self.assertAlmostEqual(wpt.position.longitude, -97.739392, places=3)
        self.assertNotEqual(wpt.position.altitude, wpt.position.altitude)
        self.assertAlmostEqual(upt.easting, 621132.815, places=3)
        self.assertAlmostEqual(upt.northing, 3360564.035, places=3)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_geo_map_py', TestGeoMap) 
