#!/usr/bin/env python

from setuptools import setup

PACKAGE_NAME = "osm_cartography"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)


setup(
    name=PACKAGE_NAME,
    version='0.2.5',
    packages=["osm_cartography"],
    data_files=[
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml")))],
    package_dir={'': 'src',},
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'geodesy',
                      'geographic_msgs',
                      'geometry_msgs',
                      'std_msgs',
                      'visualization_msgs'],
    author="Jack O'Quin",
    maintainer="Jack O'Quin",
    keywords=['ROS2'],
    description='Geographic mapping using Open Street Map data.',
    license='BSD',
    entry_points={
        'console_scripts': ['osm_client' = 'osm_cartography.nodes.osm_client:main',
                            'osm_server' = 'osm_cartography.nodes.osm_server:main',
                            'viz_osm' = 'osm_cartography.nodes.viz_osm'
        ],
    }
)
