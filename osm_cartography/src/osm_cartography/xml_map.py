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
Generate geographic information maps based on Open Street Map XML data.
"""

from __future__ import print_function

import os
from xml.etree import ElementTree

PKG_NAME = 'osm_cartography'
import roslib; roslib.load_manifest(PKG_NAME)
import rospy

import geodesy.gen_uuid

from geographic_msgs.msg import GeographicMap
from geographic_msgs.msg import KeyValue
from geographic_msgs.msg import MapFeature
from geographic_msgs.msg import UniqueID
from geographic_msgs.msg import WayPoint

def get_required_attribute(el, key):
    """ Get attribute key of element el.

    :raises:  :exc:`ValueError` if key not found
    """
    val = el.get(key)
    if val == None:
        raise ValueError('required attribute missing: ' + key)
    return val

def makeUniqueID(namespace, id):
    """Make UniqueID message for id number in OSM sub-namespace ns.

    :param namespace: OSM sub-namespace
    :type  namespace: string
    :param id: OSM identifier within that namespace
    :type  id: int or string containing an integer

    :returns: corresponding UniqueID message.
    :raises:  :exc:`ValueError`
    """
    if not namespace in {'node', 'way', 'relation'}:
        raise ValueError('invalid OSM namespace: ' + namespace)
    ns = 'http://openstreetmap.org/' + namespace + '/'
    return geodesy.gen_uuid.makeUniqueID(ns, id)

class ParseOSM:

    def __init__(self, interesting=None, ignored=None):
        self.set_tags(interesting, ignored)
            
    def get_interesting_tag(self, el):
        """Returns a KeyValue pair, if key is in interesting_tags.
    
        Returns None if no match.
        """
        #print(el.attrib)
        pair = None
        key = el.get('k')
        if key != None and key in self.interesting_tags:
            pair = KeyValue()
            pair.key = key
            pair.value = get_required_attribute(el, 'v')
        return pair
    
    def get_map(self, url, bounds):
        """Get GeographicMap from XML data.

        :param url:    Uniform Resource Locator for map.
        :param bounds: Desired bounding box for map (presently ignored).
        :returns: GeographicMap message with header not filled in.
        """

        # parse the URL
        filename = ''
        if url.startswith('file:///'):
            filename = url[7:]
        elif url.startswith('package://'):
            pkg_name, slash, pkg_path = url[10:].partition('/')
            pkg_dir = roslib.packages.get_pkg_dir(pkg_name)
            filename = pkg_dir + '/' + pkg_path
        else:
            raise ValueError('unsupported URL: ' + url)

        map = GeographicMap()
        xm = None
        try:
            f = open(filename, 'r')
            xm = ElementTree.parse(f)
        except IOError:
            raise ValueError('unable to read ' + str(url))
        except ElementTree.ParseError:
            raise ValueError('XML parse failed for ' + str(url))
        osm = xm.getroot()
    
        # get map bounds
        for el in osm.iterfind('bounds'):
            map.bounds.min_latitude =  float(get_required_attribute(el, 'minlat'))
            map.bounds.min_longitude = float(get_required_attribute(el, 'minlon'))
            map.bounds.max_latitude =  float(get_required_attribute(el, 'maxlat'))
            map.bounds.max_longitude = float(get_required_attribute(el, 'maxlon'))
    
        # get map way-point nodes
        for el in osm.iterfind('node'):
    
            way = WayPoint()
            id = el.get('id')
            if id == None:
                raise ValueError('node id missing')
            way.id = makeUniqueID('node', id)
    
            way.position.latitude = float(get_required_attribute(el, 'lat'))
            way.position.longitude = float(get_required_attribute(el, 'lon'))
            way.position.altitude = float(el.get('ele', float('nan')))
    
            for tag_list in el.iterfind('tag'):
                kv = self.get_interesting_tag(tag_list)
                if kv != None:
                    way.tags.append(kv)
    
            map.points.append(way)
    
        # get map paths
        for el in osm.iterfind('way'):
    
            feature = MapFeature()
            id = el.get('id')
            if id == None:
                raise ValueError('way id missing')
            feature.id = makeUniqueID('way', id)
    
            for nd in el.iterfind('nd'):
                way_id = get_required_attribute(nd, 'ref')
                feature.components.append(makeUniqueID('node', way_id))
    
            for tag_list in el.iterfind('tag'):
                kv = self.get_interesting_tag(tag_list)
                if kv != None:
                    if kv.value in self.ignored_values:
                        continue # skip this <way>
                    feature.tags.append(kv)
    
            map.features.append(feature)
    
        # get relations
        for el in osm.iterfind('relation'):
    
            feature = MapFeature()
            id = el.get('id')
            if id == None:
                raise ValueError('relation id missing')
            feature.id = makeUniqueID('relation', id)
    
            for mbr in el.iterfind('member'):
                mbr_type = get_required_attribute(mbr, 'type')
                if mbr_type in {'node', 'way', 'relation'}:
                    mbr_id = get_required_attribute(mbr, 'ref')
                    feature.components.append(makeUniqueID(mbr_type, mbr_id))
                else:
                    print('unknown relation member type: ' + mbr_type)
    
            for tag_list in el.iterfind('tag'):
                kv = self.get_interesting_tag(tag_list)
                if kv != None:
                    if kv.value in self.ignored_values:
                        continue # skip this <relation>
                    feature.tags.append(kv)
    
            map.features.append(feature)
    
        return map

    def set_tags(self, interesting=None, ignored=None):
        """Set interesting tags and ignored values, with reasonable
        defaults, if None provided.
        """
        if interesting is None:
            self.interesting_tags = {'access',
                                     'amenity',
                                     'boundary',
                                     'bridge',
                                     'building',
                                     'ele',
                                     'highway',
                                     'lanes',
                                     'layer',
                                     'maxheight',
                                     'maxspeed',
                                     'maxwidth',
                                     'name',
                                     'network',
                                     'oneway',
                                     'ref',
                                     'restriction',
                                     'route',
                                     'street',
                                     'tunnel',
                                     'type',
                                     'width'}
        else:
            self.interesting_tags = interesting

        if ignored is None:
            self.ignored_values = {'bridleway',
                                   'construction',
                                   'cycleway',
                                   'footway',
                                   'path',
                                   'pedestrian',
                                   'proposed',
                                   'steps'}
        else:
            self.ignored_values = ignored
