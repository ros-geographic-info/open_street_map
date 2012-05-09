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
from geographic_msgs.msg import RouteSegment
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan

class Edge():
    """
    :class:`Edge` stores graph edge data for a way point.
    """
    def __init__(self, end, heuristic=0.0):
        """Constructor.

        Collects relevant information from a route segment, providing
        convenient access to the data.

        :param end: Index of ending way point.
        :param heuristic: Distance heuristic from start to end (must
                     *not* be an over-estimate).
        """
        self.end = end
        self.h = heuristic

    def __str__(self):
        return str(self.end) + ' (' + str(self.h) + ')'

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
                    self.edges[index].append(Edge(n, heuristic=dist))
        ## Debug output:
        #for i in xrange(len(self.edges)):
        #    for k in self.edges[i]:
        #        print i, '->', k

    def planner(self, req):
        """ Plan route for nav_msgs/GetPlan service request

        :param req: nav_msgs/GetPlanRequest message.
        :returns: nav_msgs/Path message.

        :todo: implement: this stub returns an empty plan
        """
        return Path()
