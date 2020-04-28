#!/usr/bin/env python
from glob import glob
import os

from setuptools import setup

PACKAGE_NAME = "route_network"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version='0.2.5',
    packages=["route_network", "route_network.nodes"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml")))],
    package_dir={'': 'src', },
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'pyproj'],
    author="Jack O'Quin",
    maintainer="Jack O'Quin",
    keywords=['ROS2'],
    description='Route network graphing and path planning.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['plan_route = route_network.nodes.plan_route:main',
                            'route_network = route_network.nodes.route_network:main',
                            'rviz_goal = route_network.nodes.rviz_goal:main',
                            'viz_plan = route_network.nodes.viz_plan:main',
                            'viz_routes = route_network.nodes.viz_routes:main']
    }
)
