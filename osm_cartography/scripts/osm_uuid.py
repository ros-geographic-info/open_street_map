#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Austin Robot Technology
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
#  * Neither the name of Austin Robot Technology, Inc. nor the names
#    of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written
#    permission.
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
Generate UUIDs for Open Street Map data.
"""

# prepare for Python 3 migration some day
from __future__ import print_function

# the uuid package does not work with unicode strings:
#from __future__ import unicode_literals

import uuid

def generate(ns, id):
    """ Generate UUID for OSM data in name space "node", "way", or
"relation" with identifier id.

  http://tools.ietf.org/html/rfc4122.html

Matching features within the Open Street Map domain must yield the
same UUID.  The method used is RFC 4122 variant 5, computing the SHA-1
hash of a URL encoded using the map source, like this:

  http://openstreetmap.org/node/NUMBER
  http://openstreetmap.org/way/NUMBER
  http://openstreetmap.org/relation/NUMBER

Where NUMBER is the decimal representation of the OSM node, way, or
relation ID without leading zeros.

"""
    if not ns in {'node', 'way', 'relation'}:
        raise ValueError
    url = 'http://openstreetmap.org/' + ns + '/' + str(int(id))
    return uuid.uuid5(uuid.NAMESPACE_URL, url)

if __name__ == '__main__':

    # unit tests:

    x = generate('node', 1)
    y = generate('node', 0001)  # integer with leading zeros
    assert (x == y)
    assert (str(x) == 'ef362ac8-9659-5481-b954-88e9b741c8f9')

    y = generate('node', '0001') # pass integer in a string
    assert(x == y)
    assert (str(y) == 'ef362ac8-9659-5481-b954-88e9b741c8f9')
        
    z = generate('way', 1)      # same ID, different name space
    assert (x != z)
    assert (str(z) == 'b3180681-b125-5e41-bd04-3c8b046175b4')
        
    w = generate('node', 152370223) # an actual OSM node ID
    assert (w != x)
    assert (str(w) == '8e0b7d8a-c433-5c42-be2e-fbd97ddff9ac')

    # test error exceptions
    try:
        e = generate('invalid', 152370223) # invalid name space
        assert False, 'failed to raise ValueError'
    except ValueError:
        pass

    try:
        e = generate('way', 'xxx') # invalid number
        assert False, 'failed to raise ValueError'
    except ValueError:
        pass

    print('unit tests ran successfully')
