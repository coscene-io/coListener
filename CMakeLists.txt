cmake_minimum_required(VERSION 2.8.3)
project(coListener)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  src/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
