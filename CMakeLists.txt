cmake_minimum_required(VERSION 3.0.2)
project(block_localization)

# -mavx causes a lot of errors!!
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  if("$ENV{ROS_DISTRO}" STRGREATER "melodic")
    add_definitions(-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
    set(CMAKE_CXX_FLAGS "-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
  else()
    add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
    set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
  endif()
else()
  add_definitions(-std=c++17)
  set(CMAKE_CXX_FLAGS "-std=c++17")
endif()

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  tf
  map_server
  actionlib
  # pcl libraries
  pcl_ros
  # external libraries
  ndt_omp
  hdl_global_localization
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  queryMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs 
  nav_msgs 
  sensor_msgs
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)
pkg_check_modules(YAML REQUIRED yaml-cpp)
message(STATUS "YAML=${YAML_INCLUDE_DIRS} ${YAML_LIBRARIES}")

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM
  DEPENDS YAML

  CATKIN_DEPENDS
  roscpp
  rospy
  actionlib 
  tf
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime 
  message_generation
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${GTSAM_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)

# nodelets
# add_library(block_localization_nodelet src/block_localization_nodelet.cpp)
# target_link_libraries(block_localization_nodelet
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   yaml-cpp
#   gtsam
#   tbb
# )

# add_library(globalmap_server_nodelet src/globalmap_server_nodelet.cpp)
# target_link_libraries(globalmap_server_nodelet
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   yaml-cpp
#   gtsam
#   tbb
# )

# add_library(points_preprocessing_nodelet src/points_preprocessing_nodelet.cpp)
# target_link_libraries(points_preprocessing_nodelet
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   yaml-cpp
#   gtsam
#   tbb
# )

add_executable(liloc_nodelet src/LiLoc_nodelet.cpp)
target_link_libraries(liloc_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  gtsam
  tbb
)