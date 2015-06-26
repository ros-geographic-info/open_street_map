# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(route_network)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  COMPONENTS
  dynamic_reconfigure
  geographic_msgs
  geometry_msgs
  unique_id
  uuid_msgs
  visualization_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/RouteNetwork.cfg)

# what other packages will need to use this one
catkin_package(CATKIN_DEPENDS
  geographic_msgs
  geometry_msgs
  uuid_msgs
  visualization_msgs
)

# unit tests
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_planner.py)
endif()
