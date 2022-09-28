cmake_minimum_required(VERSION 3.0.2)
project(bop_ros_conversion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
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
#  INCLUDE_DIRS include
#  LIBRARIES bop_ros_conversion
  CATKIN_DEPENDS geometry_msgs message_filters nav_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  /home/pmvanderburg/noetic-husky/bop_ros_ws/src/Husky_scripts/bop_ros_conversion/src/scripts/utils
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/scripts/odom_to_tf2_broadcaster.py
  src/scripts/odom_to_static_broadcaster.py
  src/scripts/odom_to_tf2_to_ctr_broadcaster.py
  src/scripts/robot_to_zed_broadcaster.py
  src/scripts/box_mk_to_box_ctr_broadcaster.py
  src/scripts/sync_husky_and_optitrack_topics.py
  src/scripts/bag_to_csv_thijs.py
  src/scripts/tf2_to_posestamped_recorder.py
  src/scripts/tf2_lookup.py
  src/scripts/tfstamped_broadcaster.py
  src/scripts/odom_to_json_listener.py
  src/scripts/odom_to_json_listener_4.py
  src/scripts/transformerROS_listener.py
  src/scripts/rosbag_check.py
  src/scripts/rosbag_crop.py
  src/scripts/rewrite_rosmsg_header_timestamps.py
  src/scripts/rewrite_rosmsg_header_timestamps_calib.py
  src/scripts/rewrite_rostopic_names.py
  src/scripts/inspect_json.py
  src/scripts/import_test.py
  src/scripts/quat_to_euler.py
  #src/scripts/utils/quat_to_euler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
