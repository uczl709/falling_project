cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(testPCL)


#opencv:
find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Eigen
include_directories( "/usr/include/eigen3/" )
# octomap:
find_package( octomap REQUIRED )
include_directories( ${OCTOMAP_INCLUDE_DIRS} )
# PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (icp src/icp.cpp)
add_executable (ply2pcd src/ply2pcd.cpp)
target_link_libraries (icp ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES} )
target_link_libraries (ply2pcd ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES} )
