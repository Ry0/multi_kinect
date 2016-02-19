cmake_minimum_required(VERSION 2.8.3)
project(two_kinect_merge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  cmake_modules
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(Eigen REQUIRED)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS #include
#  ${PCL_COMMON_INCLUDE_DIRS}
  CATKIN_DEPENDS pcl_msgs roscpp sensor_msgs std_msgs pcl_ros tf2_sensor_msgs tf tf_conversions
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )


## Declare a C++ executable
add_executable(point_cloud_merger src/point_cloud_merger.cpp)
target_link_libraries(point_cloud_merger ${catkin_LIBRARIES})

