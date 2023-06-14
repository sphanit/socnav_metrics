cmake_minimum_required(VERSION 3.0.2)
project(socnav_metrics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  costmap_2d
  geometry_msgs
  roscpp
  tf2_ros
  tf
  tf2_eigen
  nav_msgs
)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   MetricsData.msg
   HRMetrics.msg
   AgentsData.msg
   Human.msg
   Robot.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  
   geometry_msgs
 )


catkin_package(
  CATKIN_DEPENDS rospy std_msgs  message_runtime costmap_2d geometry_msgs roscpp nav_msgs 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/metrics_api.cpp
  src/socialnavmetrics.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} socnav_metrics_generate_messages_cpp)

add_executable(metrics_node src/metrics_node.cpp)

add_dependencies(metrics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(metrics_node
  ${PROJECT_NAME}
  ${EXTERNAL_LIBS}
  ${catkin_LIBRARIES}
)

