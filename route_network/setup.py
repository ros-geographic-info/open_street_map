#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['route_network'],
    package_dir={'': 'src'},
    install_requires=['geographic_msgs', 'unique_id'],
    )

setup(**d)
