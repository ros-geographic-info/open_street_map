#!/usr/bin/env python

PKG='osm_cartography'
import roslib; roslib.load_manifest(PKG)

import unittest

#from geographic_msgs.msg import GeographicMap

from geographic_msgs.msg import GeoPoint
from geographic_msgs.msg import WayPoint
from geometry_msgs.msg import Point

from osm_cartography.way_point import *

def fromLatLong(lat, lon, alt=float('nan')):
    """Generate WayPoint from latitude, longitude and (optional) altitude.

    :returns: minimal WayPoint object just for test cases.
    """
    geo_pt = GeoPoint(latitude = lat, longitude = lon, altitude = alt)
    return WayPoint(position = geo_pt)

class TestWayPoint(unittest.TestCase):
    """Unit tests for OSM XML map parser.
    """

    def test_prc_point(self):
        ll = GeoPoint(latitude = 30.385315,
                      longitude = -97.728524,
                      altitude = 209.0)
        msg = WayPoint(position = ll)
        pt = WuPoint(msg)
        self.assertEqual(pt.way_pt, msg)
        self.assertEqual(str(pt.utm),
                         'UTM: [622159.338, 3362168.303, 209.000, 14R]')

        point_xyz = pt.toPoint()
        self.assertAlmostEqual(point_xyz.x, 622159.338, places = 3)
        self.assertAlmostEqual(point_xyz.y, 3362168.303, places = 3)
        self.assertAlmostEqual(point_xyz.z, 209.0, places = 3)

        point_xy = pt.toPointXY()
        self.assertAlmostEqual(point_xy.x, 622159.338, places = 3)
        self.assertAlmostEqual(point_xy.y, 3362168.303, places = 3)
        self.assertAlmostEqual(point_xy.z, 0.0, places = 3)

    def test_valid_points(self):
        lon = -177.0
        zone = 1
        while lon < 180.0:
            pt = WuPoint(fromLatLong(-80.0, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'C'))
            pt = WuPoint(fromLatLong(-30.385315, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'J'))
            pt = WuPoint(fromLatLong(-0.000001, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'M'))
            pt = WuPoint(fromLatLong(0.0, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'N'))
            pt = WuPoint(fromLatLong(30.385315, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'R'))
            pt = WuPoint(fromLatLong(84.0, lon))
            self.assertEqual(pt.utm.gridZone(), (zone, 'X'))
            lon += 6.0
            zone += 1

    def test_invalid_points(self):
        self.assertRaises(ValueError, WuPoint,
                          fromLatLong(90.385315, -97.728524))
        self.assertRaises(ValueError, WuPoint,
                          fromLatLong(30.385315, -197.728524))
        # this will be valid when we add UPS support for the poles:
        self.assertRaises(ValueError, WuPoint,
                          fromLatLong(-80.385315,-97.728524))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_xml_map_py', TestWayPoint) 
