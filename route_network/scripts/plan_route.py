#!/usr/bin/python
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
Plan the shortest route through a geographic information network.
"""

PKG_NAME = 'route_network'
import roslib; roslib.load_manifest(PKG_NAME)
import rospy
import sys

from route_network import planner

from geographic_msgs.msg import RouteNetwork
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan

class RoutePlannerNode():

    def __init__(self):
        """ROS node to provide a navigation plan graph for a RouteNetwork.
        """
        rospy.init_node('route_planner')
        self.graph = None
        self.planner = None

        # advertise route planning service
        self.srv = rospy.Service('get_plan', GetPlan, self.route_planner)
        self.resp = None

        # subscribe to route network
        self.sub = rospy.Subscriber('route_network', RouteNetwork,
                                    self.graph_callback)

    def graph_callback(self, graph):
        """ RouteNetwork graph message callback.
        :param graph: RouteNetwork message

        :post: graph information saved in self.graph and self.planner
        """
        self.graph = graph
        self.planner = planner.Planner(graph)

    def route_planner(self, req):
        """
        :param req: GetPlanRequest message
        :returns: GetPlanResponse message
        """
        path = Path()
        path.header.stamp = rospy.Time.now()
        if self.graph is not None:
            path.header.frame_id = self.graph.header.frame_id
            path = planner.path(req)
        self.resp = GetPlanResponse(plan = path)
        return self.resp

def main():
    node_class = RoutePlannerNode()
    try:
        rospy.spin()            # wait for messages
    except rospy.ROSInterruptException: pass
    return 0

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())
