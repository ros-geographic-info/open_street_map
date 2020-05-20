#!/usr/bin/env python
from glob import glob
import os

from setuptools import setup

PACKAGE_NAME = "osm_cartography"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version='0.2.5',
    packages=["osm_cartography", "osm_cartography.nodes"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join(SHARE_DIR, "tests"), glob(os.path.join("tests", "*.osm"))),
        (os.path.join(SHARE_DIR, "rviz"), glob(os.path.join("rviz", "*.rviz")))
    ],
    package_dir={'': 'src'},
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'pyproj'],
    author="Jack O'Quin",
    maintainer="Jack O'Quin, Bence Magyar",
    keywords=['ROS2'],
    description='Geographic mapping using Open Street Map data.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['osm_client = osm_cartography.nodes.osm_client:main',
                            'osm_server = osm_cartography.nodes.osm_server:main',
                            'viz_osm = osm_cartography.nodes.viz_osm:main'
                            ],
    }
)
