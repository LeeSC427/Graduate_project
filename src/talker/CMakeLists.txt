cmake_minimum_required(VERSION 3.0.2)
project(talker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES talker
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node 
	src/talker.cpp)

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )


