cmake_minimum_required(VERSION 3.0.2)
project(openvla_real_expr)

# add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  image_transport
  cv_bridge 
  geometry_msgs
  std_msgs
  message_generation
  tf2
  # moveit
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  # moveit_ros_move_group
  franka_msgs
  franka_description
  franka_gripper
  actionlib
  moveit_visual_tools
  # geometric_shapes
  shape_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED) 
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  openvla_action_srv.srv
  openvla_instr_srv.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  # std_msgs
)
# image_transport
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES openvla_real_expr moveit_move_group_interface franka_hw 
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    tf2 
    cv_bridge
    # moveit
    moveit_msgs 
    moveit_core 
    moveit_ros_planning 
    moveit_ros_planning_interface 
    moveit_ros_move_group 
    message_runtime
    franka_msgs
    franka_description
    franka_gripper
    actionlib
    moveit_visual_tools
    # geometric_shapes
    shape_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/eigen3
  /home/rishabh/Robot/Octo_robot/ros_ws/src/moveit/moveit_core/planning_interface/include
  /home/rishabh/Robot/Octo_robot/ros_ws/src/moveit/moveit_ros/planning_interface/move_group_interface/include
  /home/rishabh/Robot/Octo_robot/ros_ws/devel/include
  /home/rishabh/Robot/Octo_robot/ros_ws/src/franka_ros/franka_hw/include
  /home/rishabh/Robot/Octo_robot/ros_ws/src/franka_ros/franka_description
  /home/rishabh/Robot/Octo_robot/ros_ws/src/franka_ros/franka_gripper/include
  /home/rishabh/Robot/Franka/libfranka
  /home/rishabh/Robot/Octo_robot/ros_ws/src/moveit_visual_tools/include
  /opt/ros/noetic/share/
)
# ../../perception/tracking/include

## Declare a C++ library

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable

add_executable(demo_openvla_real src/demo_openvla_real.cpp)
add_executable(demo_test_action src/demo_test_action.cpp)

## Add cmake target dependencies of the executable
## same as for the library above

## Specify libraries to link a library or executable target against

Message("[vivspace] ${catkin_LIBRARIES}")

target_link_libraries(demo_openvla_real
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS} 
)

target_link_libraries(demo_test_action
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS} 
)
#   ${OpenCV_LIBRARIES}
# install(DIRECTORY launch/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

#############
## Install ##
#############

#install(PROGRAMS 
#  example/push_planner_client
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS demo_openvla_real
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS demo_test_action
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY launch
# include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # FILES_MATCHING PATTERN "*.h"
  # PATTERN ".svn" EXCLUDE
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
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
