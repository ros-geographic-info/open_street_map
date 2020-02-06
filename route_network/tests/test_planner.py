#!/usr/bin/env python

import uuid
import unittest

import geodesy.props
import geodesy.wu_point

from geographic_msgs.msg import GeoPoint
from geographic_msgs.msg import RouteNetwork
from geographic_msgs.msg import RoutePath
from geographic_msgs.msg import RouteSegment
from geographic_msgs.msg import WayPoint
from geographic_msgs.srv import GetRoutePlan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from unique_identifier_msgs.msg import UUID

suite = unittest.TestSuite()

# module to test
from route_network.planner import *

PKG_NAME = 'route_network'
PKG_URL = 'http://ros.org/wiki/' + PKG_NAME


def make_seg(start, end, oneway=False):
    """ Make RouteSegment message.

    :param start:  Initial UUID.
    :param end:    Final UUID.
    :param oneway: True if segment is one-way.
    :returns: RouteSegment message.
    """
    uu = uuid.uuid5(uuid.NAMESPACE_URL, PKG_URL + '/'
                         + str(start) + '/' + str(end))

    seg = RouteSegment(id=uuid_to_msg(str(uu)),
                       start=uuid_to_msg(start),
                       end=uuid_to_msg(end))
    if oneway:
        seg.props.append(KeyValue(key='oneway', value='yes'))
    return seg


def make_request(network, start, goal):
    return GetRoutePlan.Request(network=uuid_to_msg(network),
                               start=uuid_to_msg(start),
                               goal=uuid_to_msg(goal))


def uuid_to_msg(uid):
    return UUID(uuid=list(uuid.UUID(uid).bytes))


def msg_to_uuid(msg):
    return uuid.UUID(bytes=bytes(msg.uuid))



def make_segment(id, start, end):
    return RouteSegment(id=uuid_to_msg(id),
                        start=uuid_to_msg(start),
                        end=uuid_to_msg(end))


def make_way_point(id, lat, lon):
    w = WayPoint()
    w.id = uuid_to_msg(id)
    w.position = GeoPoint(latitude=lat, longitude=lon)
    return w


def tiny_graph():
    '''
    initialize test data: two points with segments between them
    '''
    r = RouteNetwork()

    r.points.append(make_way_point('da7c242f-2efe-5175-9961-49cc621b80b9',
                                   30.3840168, -97.72821))
    r.points.append(make_way_point('812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                   30.385729, -97.7316754))
    r.segments.append(make_segment('41b7d2be-3c37-5a78-a7ac-248d939af379',
                                   'da7c242f-2efe-5175-9961-49cc621b80b9',
                                   '812f1c08-a34b-5a21-92b9-18b2b0cf4950'))
    r.segments.append(make_segment('2df38f2c-202b-5ba5-be73-c6498cb4aafe',
                                   '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                   'da7c242f-2efe-5175-9961-49cc621b80b9'))
    return r


def tiny_oneway():
    '''
    initialize test data: two points with one segment from first to second
    '''
    r = RouteNetwork()

    r.points.append(make_way_point('da7c242f-2efe-5175-9961-49cc621b80b9',
                                 30.3840168, -97.72821))
    r.points.append(make_way_point('812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                 30.385729, -97.7316754))
    s = make_segment('41b7d2be-3c37-5a78-a7ac-248d939af379',
                    'da7c242f-2efe-5175-9961-49cc621b80b9',
                    '812f1c08-a34b-5a21-92b9-18b2b0cf4950')
    geodesy.props.put(s, 'oneway', 'yes')
    r.segments.append(s)

    return r


def triangle_graph():
    '''
    initialize test data: three points with one-way segments between them
    '''

    r = RouteNetwork()
    r.points.append(make_way_point('da7c242f-2efe-5175-9961-49cc621b80b9',
                                   30.3840168, -97.72821))
    r.points.append(make_way_point('812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                   30.385729, -97.7316754))
    r.points.append(make_way_point('2b093523-3b39-5c48-a11c-f7c65650c581',
                                   30.3849643, -97.7269564))
    s = make_segment('41b7d2be-3c37-5a78-a7ac-248d939af379',
                     'da7c242f-2efe-5175-9961-49cc621b80b9',
                     '812f1c08-a34b-5a21-92b9-18b2b0cf4950')
    geodesy.props.put(s, 'oneway', 'yes')
    r.segments.append(s)
    s = make_segment('2df38f2c-202b-5ba5-be73-c6498cb4aafe',
                     '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                     '2b093523-3b39-5c48-a11c-f7c65650c581')
    geodesy.props.put(s, 'oneway', 'yes')
    r.segments.append(s)
    s = make_segment('8f2c2df3-be73-5ba5-202b-cb4aafec6498',
                     '2b093523-3b39-5c48-a11c-f7c65650c581',
                     'da7c242f-2efe-5175-9961-49cc621b80b9')
    geodesy.props.put(s, 'oneway', 'yes')
    r.segments.append(s)
    return r


def float_range(begin, end, step):
    val = begin
    while val < end:
        yield val
        val += step


def grid_graph(min_lat, min_lon, max_lat, max_lon, step=0.001):
    """
    Generate a fully-connected rectangular grid.

    :param min_lat: Initial latitude [degrees].
    :param min_lon: Initial longitude [degrees].
    :param max_lat: Latitude limit [degrees].
    :param max_lon: Longitude limit [degrees].
    :param step: Step size [degrees].
    :returns: RouteNetwork message.
    """
    nid = uuid.uuid5(uuid.NAMESPACE_URL, PKG_URL + '/test_network')

    r = RouteNetwork(id=uuid_to_msg(str(nid)))
    prev_row = None
    for latitude in float_range(min_lat, max_lat, step):
        prev_col = None
        this_row = len(r.points)
        for longitude in float_range(min_lon, max_lon, step):
            fake_url = 'fake://point/' + str(latitude) + '/' + str(longitude)
            pt_id = uuid.uuid5(uuid.NAMESPACE_URL, fake_url)
            r.points.append(make_way_point(str(pt_id), latitude, longitude))
            if prev_col is not None:
                s = make_seg(str(prev_col), str(pt_id))
                r.segments.append(s)
                s = make_seg(str(pt_id), str(prev_col))
                r.segments.append(s)
            prev_col = pt_id
            if prev_row is not None:
                prev_id = msg_to_uuid(r.points[prev_row].id)
                s = make_seg(str(prev_id), str(pt_id))
                r.segments.append(s)
                s = make_seg(str(pt_id), str(prev_id))
                r.segments.append(s)
                prev_row += 1
        prev_row = this_row
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
        i0 = pl.points.index(str(uuid_to_msg('da7c242f-2efe-5175-9961-49cc621b80b9').uuid))
        e0 = pl.edges[i0][0]
        self.assertEqual(e0.end, 1)
        self.assertAlmostEqual(e0.h, d)
        i1 = pl.points.index(str(uuid_to_msg('812f1c08-a34b-5a21-92b9-18b2b0cf4950').uuid))
        e1 = pl.edges[i1][0]
        self.assertEqual(e1.end, 0)
        self.assertAlmostEqual(e1.h, d)
        self.assertAlmostEqual(pl.points.distance2D(i0, i1), d)

        # going forward or backward should yield a single route segment
        path = pl.planner(make_request(str(msg_to_uuid(pl.graph.id)),
                                       'da7c242f-2efe-5175-9961-49cc621b80b9',
                                       '812f1c08-a34b-5a21-92b9-18b2b0cf4950'))
        self.assertEqual(len(path.segments), 1)
        self.assertEqual(str(msg_to_uuid(path.segments[0])),
                         '41b7d2be-3c37-5a78-a7ac-248d939af379')
        path = pl.planner(make_request(str(msg_to_uuid(pl.graph.id)),
                                       '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                       'da7c242f-2efe-5175-9961-49cc621b80b9'))
        self.assertEqual(len(path.segments), 1)
        self.assertEqual(str(msg_to_uuid(path.segments[0])),
                         '2df38f2c-202b-5ba5-be73-c6498cb4aafe')

    def test_tiny_oneway_network(self):
        pl = Planner(tiny_oneway())
        self.assertEqual(len(pl.points), 2)
        self.assertEqual(len(pl.graph.segments), 1)
        d = 383.272903769
        i0 = pl.points.index(str(uuid_to_msg('da7c242f-2efe-5175-9961-49cc621b80b9').uuid))
        e0 = pl.edges[i0][0]
        self.assertEqual(e0.end, 1)
        self.assertAlmostEqual(e0.h, d)
        i1 = pl.points.index(str(uuid_to_msg('812f1c08-a34b-5a21-92b9-18b2b0cf4950').uuid))
        self.assertEqual(pl.edges[i1], [])
        self.assertAlmostEqual(pl.points.distance2D(i0, i1), d)

        # going forward should yield a single route segment
        path = pl.planner(make_request(str(msg_to_uuid(pl.graph.id)),
                                       'da7c242f-2efe-5175-9961-49cc621b80b9',
                                       '812f1c08-a34b-5a21-92b9-18b2b0cf4950'))
        self.assertEqual(len(path.segments), 1)
        self.assertEqual(str(msg_to_uuid(path.segments[0])),
                         '41b7d2be-3c37-5a78-a7ac-248d939af379')

    def test_no_path_to_goal(self):
        pl = Planner(tiny_oneway())
        req = make_request(str(msg_to_uuid(pl.graph.id)),
                           '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                           'da7c242f-2efe-5175-9961-49cc621b80b9')
        self.assertRaises(NoPathToGoalError, pl.planner, req)

    def test_starting_at_goal(self):
        pl = Planner(tiny_oneway())
        req = make_request(str(msg_to_uuid(pl.graph.id)),
                           'da7c242f-2efe-5175-9961-49cc621b80b9',
                           'da7c242f-2efe-5175-9961-49cc621b80b9')
        path = pl.planner(req)
        self.assertEqual(len(path.segments), 0)

    def test_bogus_network(self):
        pl = Planner(tiny_graph())
        req = make_request('ffffffff-ffff-ffff-ffff-ffffffffffff',
                           '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                           'da7c242f-2efe-5175-9961-49cc621b80b9')
        self.assertRaises(ValueError, pl.planner, req)

    def test_bogus_start_point(self):
        pl = Planner(tiny_graph())

        req = make_request(str(msg_to_uuid(pl.graph.id)),
                           'ffffffff-ffff-ffff-ffff-ffffffffffff',
                           'da7c242f-2efe-5175-9961-49cc621b80b9')
        self.assertRaises(ValueError, pl.planner, req)

    def test_bogus_goal(self):
        pl = Planner(tiny_graph())
        req = make_request(str(msg_to_uuid(pl.graph.id)),
                           '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                           'ffffffff-ffff-ffff-ffff-ffffffffffff')  # invalid way point
        self.assertRaises(ValueError, pl.planner, req)

    def test_triangle_routes(self):
        pl = Planner(triangle_graph())

        # going forward should yield a single route segment
        path = pl.planner(make_request(str(msg_to_uuid(pl.graph.id)),
                                       'da7c242f-2efe-5175-9961-49cc621b80b9',
                                       '812f1c08-a34b-5a21-92b9-18b2b0cf4950'))
        self.assertEqual(len(path.segments), 1)
        self.assertEqual(str(msg_to_uuid(path.segments[0])),
                         '41b7d2be-3c37-5a78-a7ac-248d939af379')

        # going backward should yield two route segments
        path = pl.planner(make_request(str(msg_to_uuid(pl.graph.id)),
                                       '812f1c08-a34b-5a21-92b9-18b2b0cf4950',
                                       'da7c242f-2efe-5175-9961-49cc621b80b9'))
        self.assertEqual(len(path.segments), 2)
        self.assertEqual(str(msg_to_uuid(path.segments[0])),
                         '2df38f2c-202b-5ba5-be73-c6498cb4aafe')
        self.assertEqual(str(msg_to_uuid(path.segments[1])),
                         '8f2c2df3-be73-5ba5-202b-cb4aafec6498')

    def test_2x2_grid(self):
        # generate a fully-connected 2x2 grid
        g = grid_graph(0.0, 0.0, 0.002, 0.002)
        pl = Planner(g)
        # self.fail(msg=str(pl))
        # all pairs of points should have a valid path
        for pt1 in g.points:
            for pt2 in g.points:
                path = pl.planner(make_request(str(msg_to_uuid(g.id)),
                                               str(msg_to_uuid(pt1.id)),
                                               str(msg_to_uuid(pt2.id))))

    def test_3x3_grid(self):
        # generate a fully-connected 3x3 grid
        g = grid_graph(-0.003, -0.003, 0.0, 0.0)
        pl = Planner(g)
        # all pairs of points should have a valid path
        for pt1 in g.points:
            for pt2 in g.points:
                path = pl.planner(make_request(str(msg_to_uuid(g.id)),
                                               str(msg_to_uuid(pt1.id)),
                                               str(msg_to_uuid(pt2.id))))

    def test_10x10_grid(self):
        # generate a fully-connected 10x10 grid
        g = grid_graph(0.0, 0.0, 0.01, 0.01)
        pl = Planner(g)
        # all pairs of points should have a valid path
        for pt1 in g.points:
            for pt2 in g.points:
                path = pl.planner(make_request(str(msg_to_uuid(g.id)),
                                               str(msg_to_uuid(pt1.id)),
                                               str(msg_to_uuid(pt2.id))))


if __name__ == '__main__':
    unittest.TextTestRunner().run(suite)
