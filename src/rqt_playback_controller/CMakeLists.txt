cmake_minimum_required(VERSION 2.8.3)
project(rqt_playback_controller)

find_package(catkin REQUIRED COMPONENTS
    rqt_gui
    rqt_gui_cpp
    actionlib_msgs
    actionlib
    rviz
    ros_action_helper
    depth_sensor_vicon_calibration
    oni_vicon_player)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    rqt_gui
    rqt_gui_cpp
    actionlib_msgs
    actionlib
    rviz
    depth_sensor_vicon_calibration
    oni_vicon_player
)

catkin_python_setup()

###########
## Build ##
###########

set(rqt_playback_controller_SOURCES
  src/rqt_playback_controller/playback_controller.cpp
)

set(rqt_playback_controller_HEADERS
  include/rqt_playback_controller/playback_controller.hpp
)

set(rqt_playback_controller_UIS
  src/rqt_playback_controller/playback_controller.ui
)

include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${catkin_INCLUDE_DIRS}
)

qt4_wrap_cpp(rqt_playback_controller_MOCS ${rqt_playback_controller_HEADERS})
qt4_wrap_ui(rqt_playback_controller_UIS_H ${rqt_playback_controller_UIS})

add_library(${PROJECT_NAME}
    ${rqt_playback_controller_SOURCES}
    ${rqt_playback_controller_HEADERS}
    ${rqt_playback_controller_MOCS}
    ${rqt_playback_controller_UIS_H}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS rqt_playback_controller
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   scripts/rqt_playback_controller
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rqt_playback_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
