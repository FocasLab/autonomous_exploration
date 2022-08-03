cmake_minimum_required(VERSION 3.0.2)
project(autonomous_exploration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  phasespace_msgs
  actionlib_msgs
  roscpp
  rospy
  actionlib
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package (Eigen3 REQUIRED NO_MODULE)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Obstacle.msg
  Target.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  autoExpl.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES autonomous_exploration
 CATKIN_DEPENDS phasespace_msgs actionlib_msgs roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(INC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${INC_PATH}
)

# Link the action server node
add_executable(scots_action_server src/scotsActionServer.cpp)

target_link_libraries(
  scots_action_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  scots_action_server
  # The one below is the name of your package!!
  ${autonomous_exploration_EXPORTED_TARGETS}
)

# Link the node action client node
add_executable(scots_action_client src/scotsActionClient.cpp)

target_link_libraries(
  scots_action_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  scots_action_client
  # The one below is the name of your package!!
  ${autonomous_exploration_EXPORTED_TARGETS}
)

# Link the node action client node
add_executable(origin_pose_publisher src/origin_pose_publisher.cpp)

target_link_libraries(
  origin_pose_publisher
  ${catkin_LIBRARIES}
)

add_dependencies(
  origin_pose_publisher
  # The one below is the name of your package!!
  ${autonomous_exploration_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_autonomous_exploration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
