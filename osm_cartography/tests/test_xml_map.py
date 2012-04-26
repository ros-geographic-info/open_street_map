#!/usr/bin/env python

PKG='osm_cartography'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

#from geographic_msgs.msg import GeographicMap

from osm_cartography import xml_map

class TestXmlMap(unittest.TestCase):
    """Unit tests for OSM XML map parser.
    """

    def test_tiny_osm_file(self):
        # :todo: deeper results verification
        pkg_dir = roslib.packages.get_pkg_dir(PKG)
        parser = xml_map.ParseOSM()
        f = open(pkg_dir + '/tests/tiny.osm', 'r')
        m = parser.get_map(f)
        self.assertEqual(len(m.points), 3)
        self.assertEqual(len(m.features), 2)

    def test_prc_osm_file(self):
        # :todo: deeper results verification
        pkg_dir = roslib.packages.get_pkg_dir(PKG)
        parser = xml_map.ParseOSM()
        f = open(pkg_dir + '/tests/prc.osm', 'r')
        m = parser.get_map(f)
        self.assertEqual(len(m.points), 986)
        self.assertEqual(len(m.features), 84)

    def test_empty_osm_file(self):
        pkg_dir = roslib.packages.get_pkg_dir(PKG)
        parser = xml_map.ParseOSM()
        f = open(pkg_dir + '/tests/empty.osm', 'r')
        self.assertRaises(ValueError, parser.get_map, f)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_xml_map_py', TestXmlMap) 
