cmake_minimum_required(VERSION 2.8)
project(test_osm)

# list catkin dependencies
set(_ROS_PKG_DEPS
    osm_cartography
    geodesy
    geographic_msgs
    route_network)

# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED COMPONENTS ${_ROS_PKG_DEPS} rostest)
catkin_python_setup()

catkin_package(CATKIN_DEPENDS ${_ROS_PKG_DEPS})

# install some useful files
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
