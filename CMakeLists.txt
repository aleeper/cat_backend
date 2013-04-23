cmake_minimum_required(VERSION 2.8.3)
project(cat_backend)

set(MSG_DEPS
  std_msgs
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs)

find_package(Boost REQUIRED system filesystem date_time program_options signals thread)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs 
  moveit_msgs 
  moveit_core 
  moveit_planners_ompl
  moveit_ros_planning_interface 
  moveit_ros_robot_interaction 
  roscpp 
  tf
  dynamic_reconfigure
  genmsg
  message_generation
  ${MSG_DEPS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   moveit_msgs
# )


generate_dynamic_reconfigure_options( 
  "cfg/Backend.cfg" 
  )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cat_backend
  CATKIN_DEPENDS geometry_msgs moveit_core moveit_msgs moveit_ros_planning_interface roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## Declare a cpp library
 add_library(cat_backend
    src/${PROJECT_NAME}/cat_backend.cpp
    src/${PROJECT_NAME}/background_processing.cpp
    src/${PROJECT_NAME}/plan_interpolator.cpp
 )
target_link_libraries(cat_backend ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Declare a cpp executable
 add_executable(cat_backend_node src/cat_backend_node.cpp)

## Add dependencies to the executable
 add_dependencies(cat_backend_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
 target_link_libraries(cat_backend_node
   cat_backend
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS cat_backend cat_backend_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cat_backend.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
