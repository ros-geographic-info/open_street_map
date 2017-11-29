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
Route network path planner.

.. _`geographic_msgs/GetRoutePlan`: http://ros.org/doc/api/geographic_msgs/html/srv/GetRoutePlan.html
.. _`geographic_msgs/RouteNetwork`: http://ros.org/doc/api/geographic_msgs/html/msg/RouteNetwork.html
.. _`geographic_msgs/RoutePath`: http://ros.org/doc/api/geographic_msgs/html/msg/RoutePath.html
.. _`uuid_msgs/UniqueID`: http://ros.org/doc/api/uuid_msgs/html/msg/UniqueID.html

"""

import numpy
import math
import geodesy.utm
import geodesy.wu_point
import rospy

from geographic_msgs.msg import RouteNetwork
from geographic_msgs.msg import RoutePath
from geographic_msgs.msg import RouteSegment
from geographic_msgs.srv import GetRoutePlan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

class PlannerError(Exception):
    """Base class for exceptions in this module."""

class NoPathToGoalError(PlannerError):
    """Exception raised when there is no path to the goal."""

class Edge():
    """
    :class:`Edge` stores graph edge data for a way point.

    :param end: Index of ending way point.
    :param seg: `uuid_msgs/UniqueID`_ of corresponding RouteSegment.
    :param heuristic: Distance heuristic from start to end (must
                      *not* be an over-estimate).
    """
    def __init__(self, end, seg, heuristic=0.0):
        """Constructor. """
        self.end = end
        self.seg = seg
        self.h = heuristic

    def __str__(self):
        return str(self.end)+' '+str(self.seg.uuid)+' ('+str(self.h)+')'

class Planner():
    """
    :class:`Planner` plans a route through a RouteNetwork.

    :param graph: `geographic_msgs/RouteNetwork`_ message.
    """
    def __init__(self, graph):
        """Constructor.

        Collects relevant information from route network message,
        providing convenient access to the data.
        """
        self._shift_to_route_within = rospy.get_param('~shift_to_route_within', 7.)
        rospy.loginfo("~shift_to_route_within: %.2f", self._shift_to_route_within)

        self.graph = graph
        self.points = geodesy.wu_point.WuPointSet(graph.points)

        # Create empty list of graph edges leaving each map point.
        self.edges = [[] for wid in xrange(len(self.points))]
        for seg in self.graph.segments:
            index = self.points.index(seg.start.uuid)
            if index is not None:
                n = self.points.index(seg.end.uuid)
                if n is not None:
                    # use 2D Euclidean distance for the heuristic
                    dist = self.points.distance2D(index, n)
                    self.edges[index].append(Edge(n, seg.id,
                                                  heuristic=dist))

        # Create a list with utm point for each point
        self.utm_points = dict()
        for p in self.points:
            self.utm_points[self.points.index(p.uuid())] = geodesy.utm.fromMsg(p.position())

    def __str__(self):
        val = '\n'
        for i in xrange(len(self.edges)):
            val += str(i) + ':\n'
            for k in self.edges[i]:
                val += '    ' + str(k) + '\n'
        return val

    def planner(self, req):
        """ Plan route from start to goal.

        :param req: `geographic_msgs/GetRoutePlan`_ request message.
        :returns: `geographic_msgs/RoutePath`_ message.
        :raises: :exc:`ValueError` if invalid request.
        :raises: :exc:`NoPathToGoalError` if goal not reachable.
        """
        # validate request parameters
        if req.network.uuid != self.graph.id.uuid: # different route network?
            raise ValueError('invalid GetRoutePlan network: '
                             + str(req.network.uuid))
        start_idx = self.points.index(req.start.uuid)
        if start_idx is None:
            raise ValueError('unknown starting point: '
                             + str(req.start.uuid))
        goal_idx = self.points.index(req.goal.uuid)
        if goal_idx is None:
            raise ValueError('unknown goal: ' + str(req.goal.uuid))

        # initialize plan
        plan = RoutePath(network=self.graph.id)
        plan.network = req.network

        # A* shortest path algorithm
        opened = [[0.0, start_idx]]
        closed = [False for wid in xrange(len(self.points))]
        closed[start_idx] = True
        backpath = [None for wid in xrange(len(self.points))]
        while True:
            if len(opened) == 0:
                raise NoPathToGoalError('No path from ' + req.start.uuid
                                        + ' to ' + req.goal.uuid)
            opened.sort()          # :todo: make search more efficient
            opened.reverse()
            h, e = opened.pop()
            if e == goal_idx:
                break
            for edge in self.edges[e]:
                e2 = edge.end
                if not closed[e2]:
                    h2 = h + edge.h
                    opened.append([h2, e2])
                    closed[e2] = True
                    backpath[e2] = [e, edge]

        # generate plan segments from backpath
        plan.segments = []
        e = goal_idx
        while backpath[e] is not None:
            plan.segments.append(backpath[e][1].seg)
            e = backpath[e][0]
        assert(e == start_idx)
        plan.segments.reverse()
        return plan

    def geo_path(self, req):
        """ Plan the shortest path between a start and a destination geopoint.

        Unlike the 'planner' method, the 'geo_path' method can receive GeoPoints out of the graph, upon such a case, the nearest segments on the OSM map are detected,
        and the planning is carried out.
        
        :pram req: The request message.
        :param req: geographic_msgs/GetGeoPath

        :return: The computed path, as well as the ids of the RouteNetwork and the start and end segments, plus the length of the path. The length is set to -1 in case of failure.
        :rtype: (geographic_msgs/GeoPoint[], uuid_msgs/UniqueID, uuid_msgs/UniqueID, uuid_msgs/UniqueID, length) 
        :raises: :exc:`ValueError` if invalid request.
        """
        # check for possible errors in request
        if math.isnan(req.start.latitude) or math.isnan(req.start.longitude):
            raise ValueError('NaN in starting point: ' + str(req.start))
        if math.isnan(req.goal.latitude) or math.isnan(req.goal.longitude):
            raise ValueError('NaN in starting point: ' + str(req.start))
        if math.isnan(req.start.altitude):
            req.start.altitude = 0.0
        if math.isnan(req.goal.altitude):
            req.goal.altitude = 0.0
        # get the nearest segment to the start point
        (start_seg, start_dist, start_lot) = self.getNearestSegment(req.start)
        if start_seg is None:
            raise ValueError('no nearest segment found for starting way point: ' + str(req.start))
        # get the nearest segment to the destination point
        (goal_seg, goal_dist, goal_lot) = self.getNearestSegment(req.goal)
        if goal_seg is None:
            raise ValueError('no nearest segment found for goal way point: ' + str(req.goal))

        # determine the path
        result = []
        try:
            (route_path, dist) = self._planner_seg(req.start, start_seg, req.goal, goal_seg)
        except NoPathToGoalError, e:
            return result, self.graph.id, start_seg.id, goal_seg.id, -1
        start_utm = geodesy.utm.fromMsg(req.start)
        goal_utm = geodesy.utm.fromMsg(req.goal)
        # copy the route to the result list and adds the start and/or goal position to the path
        if route_path.segments:
            seg = self._getSegment(route_path.segments[0])
            p = self._get_min_point(start_seg, start_lot)
            # add the start point if it is not close to the route
            dist_first_to_start = self.distance2D(p, start_utm)
            if dist_first_to_start > self._shift_to_route_within:
                result.append(req.start)
            result.append(p.toMsg())
            # add only the endpoints of the segments
            for index in xrange(len(route_path.segments)):
                seg_id = route_path.segments[index]
                seg = self._getSegment(seg_id)

                if index == 0:
                    # add the segment start point, if the plan segment and nearest segment are not equal
                    if not ((seg.end.uuid == start_seg.start.uuid and seg.start.uuid == start_seg.end.uuid) or (seg.start.uuid == start_seg.start.uuid and seg.end.uuid == start_seg.end.uuid)):
                        result.append(self.points[seg.start.uuid].position())
                if index+1 == len(route_path.segments):
                    # add the end point of the last segment, if the plan segment and nearest segment are not equal
                    if not ((seg.end.uuid == goal_seg.start.uuid and seg.start.uuid == goal_seg.end.uuid) or (seg.start.uuid == goal_seg.start.uuid and seg.end.uuid == goal_seg.end.uuid)):
                        result.append(self.points[seg.end.uuid].position())
                else:
                    result.append(self.points[seg.end.uuid].position())
            # add a perpendicular point or the nearest endpoint of the end segment
            p = self._get_min_point(goal_seg, goal_lot)
            result.append(p.toMsg())
            # add the destination point if it is not close to the route
            dist_last_to_goal = self.distance2D(p, goal_utm)
            if dist_last_to_goal > self._shift_to_route_within:
                result.append(req.goal)
        else:
            if ((start_seg.end.uuid == goal_seg.start.uuid and start_seg.start.uuid == goal_seg.end.uuid) or (start_seg.start.uuid == goal_seg.start.uuid and start_seg.end.uuid == goal_seg.end.uuid)):
                # direct connection
                result.append(req.start)
                result.append(req.goal)

        # calculate the distance
        last_utm = None
        dist = 0
        for point in result:
            if last_utm is None:
                last_utm = geodesy.utm.fromMsg(point)
            else:
                new_utm = geodesy.utm.fromMsg(point)
                dist += self.distance2D(last_utm, new_utm)
                last_utm = new_utm
        return result, self.graph.id, start_seg.id, goal_seg.id, dist

    def _planner_seg(self, start_geo_point, start_seg, goal_geo_point, goal_seg):
        """ Plan route from start to goal. The actual search algorithm to find a path is executed here.
        
        :param start_geo_point: The start position.
        :type start_geo_point: geographic_msgs/GeoPoint
        :param start_seg: The nearest segment to the point.
        :type start_seg: geographic_msgs/RouteSegment
        :param: goal_geo_point: The goal position.
        :type goal_geo_point: geographic_msgs/GeoPoint
        :param goal_seg: The nearest segment to the point.
        :type goal_seg: geographic_msgs/RouteSegment
 
        :return: The planned path between start and goal, and its length.
        :rtype: (geographic_msgs/RoutePath, float)

        :raises: :exc: ValueError if invalid request.
        :raises: :exc: NoPathToGoalError if goal not reachable.
        """
        # validate request parameters
        start__seg_start_idx = self.points.index(start_seg.start.uuid)
        start__seg_end_idx = self.points.index(start_seg.end.uuid)
        if start__seg_start_idx is None or start__seg_end_idx is None:
            raise ValueError('unknown starting segment: '
                             + str(start_seg.id))
        goal__seg_start_idx = self.points.index(goal_seg.start.uuid)
        goal__seg_end_idx = self.points.index(goal_seg.end.uuid)
        if goal__seg_start_idx is None or goal__seg_end_idx is None:
            raise ValueError('unknown goal segment: ' + str(goal_seg.id))

        # initialize plan
        plan = RoutePath(network=self.graph.id)

        # A* shortest path algorithm
        opened = []
        # add the distance from start point to the start and to the end of the nearest segment
        start_utm = geodesy.utm.fromMsg(start_geo_point)
        utm_seg_start = self.utm_points[self.points.index(start_seg.start.uuid)]
        utm_seg_end = self.utm_points[self.points.index(start_seg.end.uuid)]
        length_start2seg_start = self.distance2D(start_utm, utm_seg_start)
        length_start2seg_end = self.distance2D(start_utm, utm_seg_end)

        # distance of the last segment to the goal
        goal_utm = geodesy.utm.fromMsg(goal_geo_point)
        utm_goal_seg_start = self.utm_points[self.points.index(goal_seg.start.uuid)]
        utm_goal_seg_end = self.utm_points[self.points.index(goal_seg.end.uuid)]
        length_goal2seg_start = self.distance2D(goal_utm, utm_goal_seg_start)
        length_goal2seg_end = self.distance2D(goal_utm, utm_goal_seg_end)

        opened.append([length_start2seg_start, start__seg_start_idx])

        closed = [0 for wid in xrange(len(self.points))]
        closed[start__seg_start_idx] = -1.
        backpath = [None for wid in xrange(len(self.points))]
        reached_goal = None
        while True:
            if len(opened) == 0:
                raise NoPathToGoalError('No path from ' + str(start_geo_point) + ' to ' + str(goal_geo_point))
            opened.sort()
            h, e = opened.pop(0)
            if ((e == goal__seg_start_idx and goal__seg_start_idx != start__seg_start_idx) or (e == goal__seg_end_idx and goal__seg_end_idx != start__seg_start_idx)):
                reached_goal = e
                break
            for edge in self.edges[e]:
                e2 = edge.end
                if e2 == start__seg_end_idx:
                    h2 = length_start2seg_end
                elif e2 == start__seg_start_idx:
                    h2 = length_start2seg_start
                else:
                    h2 = h + edge.h
                if e2 == goal__seg_start_idx:
                    h2 += length_goal2seg_start
                elif e2 == goal__seg_end_idx:
                    h2 += length_goal2seg_end
                if closed[e2] == 0 or closed[e2] > h2:
                    opened.append([h2, e2])
                    closed[e2] = h2
                    backpath[e2] = [e, edge, h2]

        # generate plan segments from backpath
        plan.segments = []
        if reached_goal is None:
            raise ValueError('no path to target found')
        e = reached_goal
        dist = backpath[e][2] if backpath[e] is not None else -1
        try:
            while backpath[e] is not None:
                plan.segments.append(backpath[e][1].seg)
                e = backpath[e][0]
                # :TODO: sometimeswe we have an MemoryError
        except:
            print "Error, count of segments: ", len(plan.segments)
            raise
        assert(e == start__seg_start_idx or e == start__seg_end_idx)
        plan.segments.reverse()

        return plan, dist

    def _get_min_point(self, seg, lot):
        """ Chooses between the orthogonal projection, and the start and end points
            of the segment.

        If the given orthogonal projection lies out of the segment, the whether the start
        or end point of the segment must be chosen as the minimum point.

        :param seg: The segment.
        :type seg: geographic_msgs/RouteSegment
        :param lot: The perpendicular point to the segment.
        :type lot: geodesy.utm.UTMPoint

        :return: The perpendicular point if it is on the segment, else the start or end
                 point of the segment.
        :rtype: geodesy.utm.UTMPoint
        """
        utm_seg_start = self.utm_points[self.points.index(seg.start.uuid)]
        utm_seg_end = self.utm_points[self.points.index(seg.end.uuid)]
        length_seg = self.distance2D(utm_seg_start, utm_seg_end)
        length_lot2start = self.distance2D(lot, utm_seg_start)
        length_lot2end = self.distance2D(lot, utm_seg_end)
        if (length_lot2start <= length_seg and length_lot2end <= length_seg):
            return lot
        elif length_lot2start < length_lot2end:
            return utm_seg_start
        else:
            return utm_seg_end

    def _getSegment(self, uuid):
        """ Get the segment that corresponds to the given ID.

        :param uuid: The id of the segment.
        :type uuid: uuid_msgs/UniqueID

        :return: The segment for the given uuid.
        :rtype: geographic_msgs/RouteSegment if the segment is found, None otherwise.
        """
        for seg in self.graph.segments:
            if seg.id == uuid:
                return seg
        return None

    def _getSegmentLength(self, start_point_id, seg_id):
        """ Searches the segment with given id with given start point in a pre-cached list
            and return its length.

        :param start_point_id: The id of start point of the segment.
        :type start_point_id: uuid_msgs/UniqueID
        :param seg_id: The id of the segment.
        :type seg_id: uuid_msgs/UniqueID

        :return: Length of a segment.
        :rtype: float if the segment is found, None otherwise.
        """
        edges = self.edges[self.points.index(start_point_id.uuid)]
        for edge in edges:
            if edge.seg == seg_id:
                return edge.h
        return None

    def getNearestSegment(self, geo_point, max_dist=500.):
        """ Determine the nearest segment to the given point.

        :param geo_point: The position.
        :type geo_point:  geographic_msgs/GeoPoint
        :param max_dist: The maximal allowed distance to segment.
        :type max_dist: float

        :return: A tuple of the nearest segment, which has the minimum distance to
                 given point, the distance to the segment and the perpendicular point.
        :rtype: (geographic_msgs/RouteSegment, float, geodesy.utm.UTMPoint) or
                (None, None, None), if the distance of given point to start or
                end of the segment is greater then max_dist
        """
        utm_point = geodesy.utm.fromMsg(geo_point)
        min_dist = 999999999
        result = (None, None, None)
        for seg in self.graph.segments:
            index = self.points.index(seg.start.uuid)
            if index is not None:
                n = self.points.index(seg.end.uuid)
                if n is not None:
                    # determine the length of the segment
                    length_seg = self._getSegmentLength(seg.start, seg.id)
                    if not length_seg is None:
                      length_2start = self.distance2D(utm_point, self.utm_points[index])
                      length_2end = self.distance2D(utm_point, self.utm_points[n])
                      if length_2start < max_dist or length_2end < max_dist:
                          lot_p = self.getPerpendicularPoint2D(self.utm_points[index], self.utm_points[n], utm_point)
                          length_lot2p = self.distance2D(utm_point, lot_p)
                          length_lot2start = self.distance2D(lot_p, self.utm_points[index])
                          length_lot2end = self.distance2D(lot_p, self.utm_points[n])
                          if length_lot2start <= length_seg and length_lot2end <= length_seg:
                              cur_min = length_lot2p
                              if cur_min < min_dist:
                                  min_dist = cur_min
                                  result = (seg, min_dist, lot_p)
                          else:
                              cur_min = min(length_2start, length_2end)-1.0
                              if cur_min < min_dist:
                                  min_dist = cur_min
                                  result = (seg, min_dist, lot_p)
        return result

    def getPerpendicularPoint2D(self, utm_start, utm_end, utm_p):
        """ Returns the orthongal projection of point utm_p onto a line segment (utm_start -> utm_end)

        :param utm_start: The starting point of the line segment.
        :type utm_start: geodesy.utm.UTMPoint
        :param utm_end: The ending point of the line segment.
        :type utm_end: geodesy.utm.UTMPoint
        :param utm_p: The point.
        :type utm_p: geodesy.utm.UTMPoint

        :return: The orthogonal projection (cut point) if no errors, None otherwise.
        :rtype: geodesy.utm.UTMPoint
        """
        s = numpy.array([utm_start.easting, utm_start.northing])
        e = numpy.array([utm_end.easting, utm_end.northing])
        p = numpy.array([utm_p.easting, utm_p.northing])
        rv = e - s #direction vector of the line
        rv_2 = (rv*rv).sum()
        if rv_2 == 0.:
            raise ValueError('invalid segment length')
        try:
            lamda = ((p*rv).sum() - (s*rv).sum()) / rv_2
            lot_p = s + lamda * rv
        except:
            import traceback
            print traceback.format_exc()
        return geodesy.utm.UTMPoint(lot_p[0], lot_p[1], zone=utm_p.gridZone()[0], band=utm_p.gridZone()[1])

    @staticmethod
    def distance2D(utm1, utm2):
        """ Compute 2D Euclidean distance between two utm points.

        :param utm1: The first point.
        :type utm1: geodesy.utm.UTMPoint
        :param utm2: The second point.
        :type utm2: geodey.utm.UTMPoint

        :return: Distance in meters within the UTM XY plane. Altitudes are ignored.
        :rtype: float64
        """
        dx = utm2.easting - utm1.easting
        dy = utm2.northing - utm1.northing
        return numpy.sqrt(dx*dx + dy*dy)
