cmake_minimum_required(VERSION 3.5)
project(pcd2octomap)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  octomap_server
  pcl_ros
  roscpp
  roslib
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp 
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(pcd_octomap_converter_node
  src/pcd_octomap_converter.cpp  
)

target_link_libraries(pcd_octomap_converter_node
 ${catkin_LIBRARIES}
 ${EIGEN3_LIBRARIES}
 ${PCL_LIBRARIES}
)
