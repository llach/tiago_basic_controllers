cmake_minimum_required(VERSION 3.0.2)
project(tiago_basic_controllers)

find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_cpp
  std_msgs
)
catkin_python_setup()

catkin_package(CATKIN_DEPENDS)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/cont_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)