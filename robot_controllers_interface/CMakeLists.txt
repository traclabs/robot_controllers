cmake_minimum_required(VERSION 2.8.3)
project(robot_controllers_interface)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

#if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
#endif()

find_package(Boost REQUIRED system)
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    pluginlib
    roscpp
    robot_controllers_msgs
    craftsman_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    pluginlib
    roscpp
    robot_controllers_msgs
    craftsman_msgs
  DEPENDS
    Boost
  LIBRARIES
    robot_controllers_interface
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(robot_controllers_interface
  include/robot_controllers_interface/controller.h
  include/robot_controllers_interface/planner.h
  src/controller_loader.cpp
  src/controller_manager.cpp
  src/planner_loader.cpp
  src/planner_manager.cpp
  src/parameter_parser.cpp
)
target_link_libraries(robot_controllers_interface
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(robot_controllers_interface
  robot_controllers_msgs_generate_messages_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS robot_controllers_interface LIBRARY
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  PROGRAMS
    scripts/get_controller_state.py
    scripts/start_controller.py
    scripts/stop_controller.py
    scripts/get_planner_state.py
    scripts/start_planner.py
    scripts/stop_planner.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
