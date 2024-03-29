cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(serial_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  catkin REQUIRED
  COMPONENTS
  message_generation
  tf
)

find_package(Boost REQUIRED COMPONENTS 
    system
    thread
)


add_message_files(
  FILES
  Cmd_uav.msg
  State_uav.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)  
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

add_executable(comm_read_write
  src/serial_comm.cpp
  src/serial_ros_node.cpp
)

# add_dependencies(comm_read_write ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(comm_read_write ${catkin_LIBRARIES} ${self_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(comm_read_write ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${Boost_LIBRARIES})
