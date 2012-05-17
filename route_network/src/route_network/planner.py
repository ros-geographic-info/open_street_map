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
.. module:: planner: Route network path planner.

"""

PKG = 'route_network'
import roslib; roslib.load_manifest(PKG)
import rospy

import geodesy.wu_point

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
    """
    def __init__(self, end, seg, heuristic=0.0):
        """Constructor.

        Collects relevant information from a route segment, providing
        convenient access to the data.

        :param end: Index of ending way point.
        :param seg: UniqueID of corresponding RouteSegment.
        :param heuristic: Distance heuristic from start to end (must
                     *not* be an over-estimate).
        """
        self.end = end
        self.seg = seg
        self.h = heuristic

    def __str__(self):
        return str(self.end)+' '+str(self.seg.uuid)+' ('+str(self.h)+')'

class Planner():
    """
    :class:`Planner` plans a route through a RouteNetwork.
    """
    def __init__(self, graph):
        """Constructor.

        Collects relevant information from route network message,
        providing convenient access to the data.

        :param graph: RouteNetwork message
        """
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

    def __str__(self):
        val = '\n'
        for i in xrange(len(self.edges)):
            val += str(i) + ':\n'
            for k in self.edges[i]:
                val += '    ' + str(k) + '\n'
        return val

    def planner(self, req):
        """ Plan route from start to goal.

        :param req: geographic_msgs/GetRoutePlanRequest message.
        :returns: geographic_msgs/RoutePath message.
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
        open = [[0.0, start_idx]]
        closed = [False for wid in xrange(len(self.points))]
        closed[start_idx] = True
        backpath = [None for wid in xrange(len(self.points))]
        while True:
            if len(open) == 0:
                raise NoPathToGoalError('No path from ' + req.start.uuid
                                        + ' to ' + req.goal.uuid)
            open.sort()         # :todo: make search more efficient
            open.reverse()
            h, e = open.pop()
            if e == goal_idx:
                break
            for edge in self.edges[e]:
                e2 = edge.end
                if not closed[e2]:
                    h2 = h + edge.h
                    open.append([h2, e2])
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
