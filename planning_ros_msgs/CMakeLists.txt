cmake_minimum_required(VERSION 2.8.3)

project(planning_ros_msgs)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_install()

cs_export()
