cmake_minimum_required(VERSION 2.8.3)
project(filter_tf)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  tf2_ros
  geometry_msgs
  )

catkin_package()

install(PROGRAMS
  scripts/filter_tf.py
  scripts/test_filter_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
