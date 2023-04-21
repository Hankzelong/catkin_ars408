cmake_minimum_required(VERSION 3.0.2)
project(ars408)



find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rosmsg
  rospy
  std_msgs
)



add_message_files(
   FILES
   Cluster.msg
   ClusterList.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ars408
  CATKIN_DEPENDS message_generation message_runtime roscpp rosmsg rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  lib
  ${catkin_LIB_DIRS}
)

add_executable(main src/main.cpp)
target_link_libraries(main
        ${catkin_LIBRARIES}
        controlcan
)

add_dependencies(main ars408_gencpp)


