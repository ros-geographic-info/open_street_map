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

.. _`geographic_msgs/BoundingBox`: http://ros.org/doc/api/geographic_msgs/html/msg/BoundingBox.html
.. _`geographic_msgs/GeographicMap`: http://ros.org/doc/api/geographic_msgs/html/msg/GeographicMap.html
.. _`geographic_msgs/KeyValue`: http://ros.org/doc/api/geographic_msgs/html/msg/KeyValue.html
.. _`geographic_msgs/UniqueID`: http://ros.org/doc/api/geographic_msgs/html/msg/UniqueID.html

"""

from __future__ import print_function

from xml.etree import ElementTree

PKG_NAME = 'osm_cartography'
import roslib; roslib.load_manifest(PKG_NAME)

import unique_id
from geodesy import bounding_box

from geographic_msgs.msg import GeographicMap
from geographic_msgs.msg import KeyValue
from geographic_msgs.msg import MapFeature
from geographic_msgs.msg import WayPoint

try:
    from geographic_msgs.msg import UniqueID
except ImportError:
    from uuid_msgs.msg import UniqueID

def get_required_attribute(el, key):
    """ Get attribute key of element *el*.

    :raises:  :exc:`ValueError` if key not found
    """
    val = el.get(key)
    if val == None:
        raise ValueError('required attribute missing: ' + key)
    return val

def makeOsmUniqueID(namespace, el_id):
    """Make UniqueID message for *el_id* number in OSM sub-namespace *namespace*.

    :param namespace: OSM sub-namespace
    :type  namespace: string
    :param el_id: OSM identifier within that namespace
    :type  el_id: int or string containing an integer

    :returns: corresponding `geographic_msgs/UniqueID`_ message.
    :raises:  :exc:`ValueError`
    """
    if not namespace in set(['node', 'way', 'relation']):
        raise ValueError('invalid OSM namespace: ' + namespace)
    ns = 'http://openstreetmap.org/' + namespace + '/'
    return unique_id.toMsg(unique_id.fromURL(ns + str(el_id)))

def get_tag(el):
    """ :returns: `geographic_msgs/KeyValue`_ message for `<tag>` *el* if any, None otherwise. """
    pair = None
    key = el.get('k')
    if key != None:
        pair = KeyValue()
        pair.key = key
        pair.value = get_required_attribute(el, 'v')
        return pair
    
def get_osm(url, bounds):
    """Get `geographic_msgs/GeographicMap`_ from Open Street Map XML data.

    The latitude and longitude of the bounding box returned may differ
    from the requested bounds.

    :param url:    Uniform Resource Locator for map.
    :param bounds: Desired `geographic_msgs/BoundingBox`_ for map (presently ignored).
    :returns: `geographic_msgs/GeographicMap`_ message (header not filled in).
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

    gmap = GeographicMap(id = unique_id.toMsg(unique_id.fromURL(url)))
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
        minlat = float(get_required_attribute(el, 'minlat'))
        minlon = float(get_required_attribute(el, 'minlon'))
        maxlat = float(get_required_attribute(el, 'maxlat'))
        maxlon = float(get_required_attribute(el, 'maxlon'))
        gmap.bounds = bounding_box.makeBounds2D(minlat, minlon, maxlat, maxlon)

    # get map way-point nodes
    for el in osm.iterfind('node'):

        way = WayPoint()
        el_id = el.get('id')
        if el_id is None:
            raise ValueError('node id missing')
        way.id = makeOsmUniqueID('node', el_id)

        way.position.latitude = float(get_required_attribute(el, 'lat'))
        way.position.longitude = float(get_required_attribute(el, 'lon'))
        way.position.altitude = float(el.get('ele', float('nan')))

        for tag_list in el.iterfind('tag'):
            kv = get_tag(tag_list)
            if kv != None:
                way.props.append(kv)

        gmap.points.append(way)

    # get map paths
    for el in osm.iterfind('way'):

        feature = MapFeature()
        el_id = el.get('id')
        if el_id is None:
            raise ValueError('way id missing')
        feature.id = makeOsmUniqueID('way', el_id)

        for nd in el.iterfind('nd'):
            way_id = get_required_attribute(nd, 'ref')
            feature.components.append(makeOsmUniqueID('node', way_id))

        for tag_list in el.iterfind('tag'):
            kv = get_tag(tag_list)
            if kv != None:
                feature.props.append(kv)

        gmap.features.append(feature)

    # get relations
    for el in osm.iterfind('relation'):

        feature = MapFeature()
        el_id = el.get('id')
        if el_id is None:
            raise ValueError('relation id missing')
        feature.id = makeOsmUniqueID('relation', el_id)

        for mbr in el.iterfind('member'):
            mbr_type = get_required_attribute(mbr, 'type')
            if mbr_type in set(['node', 'way', 'relation']):
                mbr_id = get_required_attribute(mbr, 'ref')
                feature.components.append(makeOsmUniqueID(mbr_type, mbr_id))
            else:
                print('unknown relation member type: ' + mbr_type)

        for tag_list in el.iterfind('tag'):
            kv = get_tag(tag_list)
            if kv != None:
                feature.props.append(kv)

        gmap.features.append(feature)

    return gmap

interesting_tags = set(['access',
                        'amenity',
                        'boundary',
                        'bridge',
                        'building',
                        'ele',
                        'highway',
                        'landuse',
                        'lanes',
                        'layer',
                        'maxheight',
                        'maxspeed',
                        'maxwidth',
                        'name',
                        'network',
                        'oneway',
                        'railway',
                        'ref',
                        'restriction',
                        'route',
                        'street',
                        'tunnel',
                        'type',
                        'width'])


ignored_values = set(['bridleway',
                      'construction',
                      'cycleway',
                      'footway',
                      'path',
                      'pedestrian',
                      'proposed',
                      'steps'])
