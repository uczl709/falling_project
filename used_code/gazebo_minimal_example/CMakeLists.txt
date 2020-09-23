CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(gazebo_minimal_example)

# DEPENDENCIES
CMAKE_POLICY(SET CMP0054 NEW)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(gazebo REQUIRED QUIET)
FIND_PACKAGE(
  catkin
  REQUIRED
    roscpp
    rospy
    rosbag
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    image_transport
    cv_bridge
)
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# CATKING PACKAGE
CATKIN_PACKAGE(
  CATKIN_DEPENDS
    roscpp
    rospy
    rosbag
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    image_transport
    cv_bridge
  LIBRARIES
    gazebo_minimal_example
)

# Gazebo plugins
#link_directories(${GAZEBO_LIBRARY_DIRS})
#include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

#SET(DEPS ${catkin_LIBRARIES} ${GAZEBO_LIBRARY_DIRS})
SET(DEPS ${catkin_LIBRARIES})
ADD_LIBRARY(my_depth_plugin SHARED src/depth_plugin.cpp)
TARGET_LINK_LIBRARIES(my_depth_plugin ${DEPS})

ADD_LIBRARY(my_pose_plugin SHARED src/pose_plugin.cpp)
TARGET_LINK_LIBRARIES(my_pose_plugin ${DEPS})

ADD_LIBRARY(my_poseinobj_plugin SHARED src/poseinobj_plugin.cpp)
TARGET_LINK_LIBRARIES(my_poseinobj_plugin ${DEPS})

ADD_LIBRARY(my_duck_plugin SHARED src/duck_plugin.cpp)
TARGET_LINK_LIBRARIES(my_duck_plugin ${DEPS})
