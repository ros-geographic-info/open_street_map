#!/usr/bin/env python

PKG='route_network'
import roslib; roslib.load_manifest(PKG)

import unittest

import geodesy.wu_point

from geographic_msgs.msg import RouteNetwork
from geographic_msgs.msg import RouteSegment
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan

# module to test
from route_network.planner import *

class TestPlanner(unittest.TestCase):
    """Unit tests for planner module.
    """

    def test_empty_route_network(self):
        pl = Planner(RouteNetwork())
        self.assertEqual(len(pl.points), 0)
        self.assertEqual(len(pl.graph.segments), 0)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_geo_map_py', TestPlanner) 
