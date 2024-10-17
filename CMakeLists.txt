cmake_minimum_required(VERSION 3.0.2)
project(fusion)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sgtdv_msgs
  roslib
  tf
  tf_conversions
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../
)

add_executable(${PROJECT_NAME} src/fusion.cpp src/fusion_node.cpp src/fusion_synch.cpp src/fusion_kf.cpp)

target_link_libraries(fusion
  ${catkin_LIBRARIES}
)
