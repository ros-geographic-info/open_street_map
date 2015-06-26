# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(osm_cartography)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  COMPONENTS
  dynamic_reconfigure
  geographic_msgs
  geometry_msgs
  unique_id
  uuid_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/VisOSM.cfg)

# what other packages will need to use this one
catkin_package(
#    CATKIN-DEPENDS geographic_msgs geometry_msgs unique_id uuid_msgs
)

# unit tests
catkin_add_nosetests(tests/test_geo_map.py)
catkin_add_nosetests(tests/test_xml_map.py)

