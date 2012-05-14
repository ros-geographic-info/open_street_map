#!/usr/bin/env python

PKG='route_network'
import roslib; roslib.load_manifest(PKG)

import unittest

import geodesy.props
import geodesy.wu_point

from geographic_msgs.msg import GeoPoint
from geographic_msgs.msg import RouteNetwork
from geographic_msgs.msg import RoutePath
from geographic_msgs.msg import RouteSegment
from geographic_msgs.msg import UniqueID
from geographic_msgs.msg import WayPoint
from geographic_msgs.srv import GetRoutePlanRequest
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

# module to test
from route_network.planner import *

def makeRequest(network, start, goal):
    return GetRoutePlanRequest(UniqueID(uuid=network),
                               UniqueID(uuid=start),
                               UniqueID(uuid=goal))

def makeSegment(id, start, end):
    return RouteSegment(id = UniqueID(id),
                        start = UniqueID(start),
                        end = UniqueID(end))

def makeWayPoint(id, lat, lon):
    w = WayPoint()
    w.id = UniqueID(id)
    w.position = GeoPoint(latitude=lat, longitude=lon)
    return w

def tiny_graph():
    'initialize test data: two points with segments between them'
    r = RouteNetwork()
    r.points.append(makeWayPoint('da7c242f-2efe-5175-9961-49cc621b80b9',
                                 30.3840168, -97.72821))
    r.points.append(makeWayPoint('812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                 30.385729, -97.7316754))
    r.segments.append(makeSegment('41b7d2be-3c37-5a78-a7ac-248d939af379',
                                  'da7c242f-2efe-5175-9961-49cc621b80b9',
                                  '812f1c08-a34b-5a21-92b9-18b2b0cf4950'))
    r.segments.append(makeSegment('2df38f2c-202b-5ba5-be73-c6498cb4aafe',
                                  '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                  'da7c242f-2efe-5175-9961-49cc621b80b9'))
    return r

def tiny_oneway():
    'initialize test data: two points with one segment from first to second'
    r = RouteNetwork()
    r.points.append(makeWayPoint('da7c242f-2efe-5175-9961-49cc621b80b9',
                                 30.3840168, -97.72821))
    r.points.append(makeWayPoint('812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                 30.385729, -97.7316754))
    s = makeSegment('41b7d2be-3c37-5a78-a7ac-248d939af379',
                    'da7c242f-2efe-5175-9961-49cc621b80b9',
                    '812f1c08-a34b-5a21-92b9-18b2b0cf4950')
    geodesy.props.put(s, 'oneway', 'yes')
    r.segments.append(s)

    return r

def triangle_graph():
    'initialize test data: three points with one-way segments between them'
    r = RouteNetwork()
    r.points.append(makeWayPoint('da7c242f-2efe-5175-9961-49cc621b80b9',
                                 30.3840168, -97.72821))
    r.points.append(makeWayPoint('812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                 30.385729, -97.7316754))
    r.points.append(makeWayPoint('2b093523-3b39-5c48-a11c-f7c65650c581',
                                 30.3849643,-97.7269564))
    s = makeSegment('41b7d2be-3c37-5a78-a7ac-248d939af379',
                    'da7c242f-2efe-5175-9961-49cc621b80b9',
                    '812f1c08-a34b-5a21-92b9-18b2b0cf4950')
    geodesy.props.put(s, 'oneway', 'yes')
    r.segments.append(s)
    s = makeSegment('2df38f2c-202b-5ba5-be73-c6498cb4aafe',
                    '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                    '2b093523-3b39-5c48-a11c-f7c65650c581')
    geodesy.props.put(s, 'oneway', 'yes')
    r.segments.append(s)
    s = makeSegment('8f2c2df3-be73-5ba5-202b-cb4aafec6498',
                    '2b093523-3b39-5c48-a11c-f7c65650c581',
                    'da7c242f-2efe-5175-9961-49cc621b80b9')
    geodesy.props.put(s, 'oneway', 'yes')
    r.segments.append(s)
    return r

class TestPlanner(unittest.TestCase):
    """Unit tests for planner module.
    """

    def test_empty_route_network(self):
        pl = Planner(RouteNetwork())
        self.assertEqual(len(pl.points), 0)
        self.assertEqual(len(pl.graph.segments), 0)

    def test_tiny_route_network(self):
        pl = Planner(tiny_graph())
        self.assertEqual(len(pl.points), 2)
        self.assertEqual(len(pl.graph.segments), 2)
        d = 383.272903769
        i0 = pl.points.index('da7c242f-2efe-5175-9961-49cc621b80b9')
        e0 = pl.edges[i0][0]
        self.assertEqual(e0.end, 1)
        self.assertAlmostEqual(e0.h, d)
        i1 = pl.points.index('812f1c08-a34b-5a21-92b9-18b2b0cf4950')
        e1 = pl.edges[i1][0]
        self.assertEqual(e1.end, 0)
        self.assertAlmostEqual(e1.h, d)
        self.assertAlmostEqual(pl.points.distance2D(i0, i1), d)

        # going forward or backward should yield a single route segment
        path = pl.planner(makeRequest(pl.graph.id.uuid,
                                      'da7c242f-2efe-5175-9961-49cc621b80b9',
                                      '812f1c08-a34b-5a21-92b9-18b2b0cf4950'))
        self.assertEqual(len(path.segments), 1)
        self.assertEqual(path.segments[0].uuid,
                         '41b7d2be-3c37-5a78-a7ac-248d939af379')
        path = pl.planner(makeRequest(pl.graph.id.uuid,
                                      '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                      'da7c242f-2efe-5175-9961-49cc621b80b9'))
        self.assertEqual(len(path.segments), 1)
        self.assertEqual(path.segments[0].uuid,
                         '2df38f2c-202b-5ba5-be73-c6498cb4aafe')

    def test_tiny_oneway_network(self):
        pl = Planner(tiny_oneway())
        self.assertEqual(len(pl.points), 2)
        self.assertEqual(len(pl.graph.segments), 1)
        d = 383.272903769
        i0 = pl.points.index('da7c242f-2efe-5175-9961-49cc621b80b9')
        e0 = pl.edges[i0][0]
        self.assertEqual(e0.end, 1)
        self.assertAlmostEqual(e0.h, d)
        i1 = pl.points.index('812f1c08-a34b-5a21-92b9-18b2b0cf4950')
        self.assertEqual(pl.edges[i1], [])
        self.assertAlmostEqual(pl.points.distance2D(i0, i1), d)

        # going forward should yield a single route segment
        path = pl.planner(makeRequest(pl.graph.id.uuid,
                                      'da7c242f-2efe-5175-9961-49cc621b80b9',
                                      '812f1c08-a34b-5a21-92b9-18b2b0cf4950'))
        self.assertEqual(len(path.segments), 1)
        self.assertEqual(path.segments[0].uuid,
                         '41b7d2be-3c37-5a78-a7ac-248d939af379')

    def test_no_path_to_goal(self):
        pl = Planner(tiny_oneway())
        req = makeRequest(pl.graph.id.uuid,
                          '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                          'da7c242f-2efe-5175-9961-49cc621b80b9')
        self.assertRaises(NoPathToGoalError, pl.planner, req)

    def test_starting_at_goal(self):
        pl = Planner(tiny_oneway())
        req = makeRequest(pl.graph.id.uuid,
                          'da7c242f-2efe-5175-9961-49cc621b80b9',
                          'da7c242f-2efe-5175-9961-49cc621b80b9')
        path = pl.planner(req)
        self.assertEqual(len(path.segments), 0)

    def test_bogus_network(self):
        pl = Planner(tiny_graph())
        req = makeRequest('deadbeef',
                          '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                          'da7c242f-2efe-5175-9961-49cc621b80b9')
        self.assertRaises(ValueError, pl.planner, req)

    def test_bogus_start_point(self):
        pl = Planner(tiny_graph())
        req = makeRequest(pl.graph.id.uuid,
                          'deadbeef',
                          'da7c242f-2efe-5175-9961-49cc621b80b9')
        self.assertRaises(ValueError, pl.planner, req)

    def test_bogus_goal(self):
        pl = Planner(tiny_graph())
        req = makeRequest(pl.graph.id.uuid,
                          '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                          'deadbeef') # invalid way point
        self.assertRaises(ValueError, pl.planner, req)

    def test_triangle_routes(self):
        pl = Planner(triangle_graph())

        # going forward should yield a single route segment
        path = pl.planner(makeRequest(pl.graph.id.uuid,
                                      'da7c242f-2efe-5175-9961-49cc621b80b9',
                                      '812f1c08-a34b-5a21-92b9-18b2b0cf4950'))
        self.assertEqual(len(path.segments), 1)
        self.assertEqual(path.segments[0].uuid,
                         '41b7d2be-3c37-5a78-a7ac-248d939af379')

        # going backward should yield two route segments
        path = pl.planner(makeRequest(pl.graph.id.uuid,
                                      '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                      'da7c242f-2efe-5175-9961-49cc621b80b9'))
        self.assertEqual(len(path.segments), 2)
        self.assertEqual(path.segments[0].uuid,
                         '2df38f2c-202b-5ba5-be73-c6498cb4aafe')
        self.assertEqual(path.segments[1].uuid,
                         '8f2c2df3-be73-5ba5-202b-cb4aafec6498')
        

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_geo_map_py', TestPlanner) 
