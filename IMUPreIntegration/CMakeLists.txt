cmake_minimum_required(VERSION 3.0.2)
project(imu_preintegration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/imu_pre_integration_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
