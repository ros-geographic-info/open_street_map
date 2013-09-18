cmake_minimum_required(VERSION 2.8)
project(route_network)

# list catkin dependencies
set(_ROS_PKG_DEPS
    dynamic_reconfigure
    geodesy
    geographic_msgs
    geometry_msgs
    nav_msgs
    rospy
    visualization_msgs)


# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED COMPONENTS ${_ROS_PKG_DEPS})
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_python_setup()

# auto-generate dynamic reconfiguration GUI: before catkin_package()
# and after catkin_python_setup().
generate_dynamic_reconfigure_options(cfg/RouteNetwork.cfg)

catkin_package(CATKIN_DEPENDS ${_ROS_PKG_DEPS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

# unit tests
if (CATKIN_ENABLE_TESTING)

  catkin_add_nosetests(tests/test_planner.py)

  # check all the launch files
  #rosbuild_add_roslaunch_check(launch)

endif (CATKIN_ENABLE_TESTING)
