# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
cmake_minimum_required(VERSION 3.16...3.26)
project(kiss_icp VERSION 0.3.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build the core KISS-ICP C++ library
add_subdirectory(cpp/kiss_icp)

# Detect ROS version
set(ROS_VERSION $ENV{ROS_VERSION})
message(STATUS "Detected ROS_VERSION: '${ROS_VERSION}'")

if("${ROS_VERSION}" STREQUAL "1")
  message(STATUS "Building KISS-ICP for ROS 1")
  # ---- ROS 1 build ----
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
  )
  catkin_package()

  add_executable(odometry_node ros1/OdometryServer.cpp)
  target_include_directories(odometry_node PRIVATE ros1 ${catkin_INCLUDE_DIRS})
  target_link_libraries(odometry_node kiss_icp::pipeline ${catkin_LIBRARIES})
  install(TARGETS odometry_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

elseif("${ROS_VERSION}" STREQUAL "2")
  message(STATUS "Building KISS-ICP for ROS 2")
  # ---- ROS 2 build ----
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common filters)

  # Build the ROS 2 odometry node
  add_executable(odometry_node ros2/OdometryServer.cpp)
  target_include_directories(odometry_node PRIVATE
    ros2
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(odometry_node
    kiss_icp::pipeline
    ${PCL_LIBRARIES}
  )
  ament_target_dependencies(odometry_node
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
    message_filters
    pcl_conversions
  )

  # Install targets
  install(TARGETS odometry_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
  install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )

  ament_package()

else()
  message(STATUS "ROS_VERSION not set or unrecognized ('${ROS_VERSION}'), building core library only")
endif()