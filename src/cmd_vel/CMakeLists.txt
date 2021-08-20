cmake_minimum_required(VERSION 3.0.2)
project(cmd_vel)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cmd_vel
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/cmd_vel.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )