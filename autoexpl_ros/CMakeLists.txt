cmake_minimum_required(VERSION 3.0.2)
project(autoexpl_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  std_srvs
  geometry_msgs
  autoexpl_msgs
  visualization_msgs
  roscpp
  rospy
  actionlib
  tf2_ros
  tf2
  pcl_ros
  pcl_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
 INCLUDE_DIRS
  include
#  LIBRARIES autoexpl_ros
 CATKIN_DEPENDS
  std_srvs
  geometry_msgs
  autoexpl_msgs
  visualization_msgs
  roscpp
  rospy
  actionlib
  tf2_ros
  tf2
  pcl_ros
  pcl_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Link the nodes
add_executable(scots_action_server src/scotsActionServer.cpp)
add_executable(scots_action_client src/scotsActionClient.cpp)
add_executable(origin_pose_publisher src/origin_pose_publisher.cpp)
add_executable(robot_pose_publisher src/robot_pose_publisher.cpp)
add_executable(get_dynamic_location src/get_dynamic_location.cpp)

target_link_libraries(
  scots_action_server
  ${catkin_LIBRARIES}
)

target_link_libraries(
  scots_action_client
  ${catkin_LIBRARIES}
)

target_link_libraries(
  origin_pose_publisher
  ${catkin_LIBRARIES}
)

target_link_libraries(
  robot_pose_publisher
  ${catkin_LIBRARIES}
)

target_link_libraries(
  get_dynamic_location
  ${catkin_LIBRARIES}
)
# add_dependencies(
#   scots_action_server
#   scots_action_client
#   origin_pose_publisher
#   robot_pose_publisher
#   # The one below is the name of your package!!
#   ${autoexpl_ros_EXPORTED_TARGETS}
# )

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
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_autoexpl_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
