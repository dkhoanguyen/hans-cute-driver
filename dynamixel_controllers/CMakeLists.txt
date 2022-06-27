# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_controller)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy
)

install(
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)