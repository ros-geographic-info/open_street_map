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

        # test GeoMapPoints iterator with empty list
        gpts = GeoMapPoints(gm)
        i = 0
        for w in gpts:
            self.fail(msg='there are no points in this map')
            i += 1
        self.assertEqual(i, 0)
        with self.assertRaises(KeyError):
            x = gpts['da7c242f-2efe-5175-9961-49cc621b80b9']

    def test_empty_map_features(self):
        gm = GeoMap(GeographicMap())
        self.assertEqual(gm.n_features, 0)

        # test GeoMapFeatures iterator with empty list
        gf = GeoMapFeatures(gm)
        i = 0
        for f in gf:
            self.fail(msg='there are no features in this map')
            i += 1
        self.assertEqual(i, 0)
        with self.assertRaises(KeyError):
            x = gf['da7c242f-2efe-5175-9961-49cc621b80b9']

    def test_tiny_map(self):
        parser = osm_cartography.xml_map.ParseOSM()
        gm = GeoMap(parser.get_map('package://osm_cartography/tests/tiny.osm',
                                   BoundingBox()))
        self.assertEqual(gm.n_points, 3)

        # expected way point IDs and UTM coordinates
        uuids = ['da7c242f-2efe-5175-9961-49cc621b80b9',
                 '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                 '6f0606f6-a776-5940-b5ea-5e889b32c712']
        eastings = [622191.124, 621856.023, 622294.785]
        northings = [3362024.764, 3362210.790, 3362320.569]

        # test GeoMapPoints iterator
        gpts = GeoMapPoints(gm)
        i = 0
        for w in gpts:
            self.assertEqual(w.uuid, uuids[i])
            self.assertEqual(gpts[uuids[i]].uuid, uuids[i])
            self.assertAlmostEqual(w.utm.easting, eastings[i], places=3)
            self.assertAlmostEqual(w.utm.northing, northings[i], places=3)
            i += 1
        self.assertEqual(i, 3)
        self.assertEqual(len(gpts), 3)

    def test_tiny_map_features(self):
        parser = osm_cartography.xml_map.ParseOSM()
        gm = GeoMap(parser.get_map('package://osm_cartography/tests/tiny.osm',
                                   BoundingBox()))
        self.assertEqual(gm.n_features, 2)

        # expected feature IDs
        uuids = ['8e8b355f-f1e8-5d82-827d-91e688e807e4',
                 '199dd143-8309-5401-9728-6ca5e1c6e235']

        # test GeoMapFeatures iterator
        gf = GeoMapFeatures(gm)
        i = 0
        for f in gf:
            self.assertEqual(f.id.uuid, uuids[i])
            i += 1
        self.assertEqual(i, 2)
        self.assertEqual(len(gf), 2)

    def test_prc_map(self):
        parser = osm_cartography.xml_map.ParseOSM()
        gm = GeoMap(parser.get_map('package://osm_cartography/tests/prc.osm',
                                   BoundingBox()))
        self.assertEqual(gm.n_features, 84)

        gpts = GeoMapPoints(gm)
        self.assertEqual(len(gpts), 986)
        self.assertFalse('00000000-c433-5c42-be2e-fbd97ddff9ac' in gpts)
        uu = '8e0b7d8a-c433-5c42-be2e-fbd97ddff9ac'
        self.assertTrue(uu in gpts)
        wpt = gpts[uu]
        self.assertEqual(wpt.uuid, uu)
        self.assertAlmostEqual(wpt.position.latitude, 30.370945, places=3)
        self.assertAlmostEqual(wpt.position.longitude, -97.739392, places=3)
        self.assertNotEqual(wpt.position.altitude, wpt.position.altitude)
        self.assertAlmostEqual(wpt.utm.easting, 621132.815, places=3)
        self.assertAlmostEqual(wpt.utm.northing, 3360564.035, places=3)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_geo_map_py', TestGeoMap) 
