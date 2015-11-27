cmake_minimum_required(VERSION 2.8.3)
project(robospect_platform_mission_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  robospect_msgs
)

catkin_python_setup()

catkin_package()

