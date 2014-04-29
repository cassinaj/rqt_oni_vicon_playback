cmake_minimum_required(VERSION 2.8.3)
project(depth_sensor_vicon_calibration)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    actionlib_msgs
    actionlib
    interactive_markers
    simple_object_tracker)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################
add_action_files(
  DIRECTORY action
  FILES
  GlobalCalibration.action
  ContinueGlobalCalibration.action
)

generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES depth_sensor_vicon_calibration
  CATKIN_DEPENDS actionlib_msgs actionlib interactive_markers simple_object_tracker
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

set(headers include/depth_sensor_vicon_calibration/depth_sensor_vicon_calibration.hpp)

add_library(depth_sensor_vicon_calibration
   src/depth_sensor_vicon_calibration.cpp
   ${headers}
)

target_link_libraries(depth_sensor_vicon_calibration
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS depth_sensor_vicon_calibration depth_sensor_vicon_calibration_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_depth_sensor_vicon_calibration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
