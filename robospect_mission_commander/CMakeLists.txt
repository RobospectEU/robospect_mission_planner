cmake_minimum_required(VERSION 2.8.3)
project(robospect_mission_commander)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_robospect_mission_commander
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/robospect_mission_commander/rqt_mission_commander.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

