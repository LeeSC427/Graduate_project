cmake_minimum_required(VERSION 3.0.2)
project(pointer_game)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pointer_game
  CATKIN_DEPENDS cv_bridge roscpp std_msgs
  DEPENDS system_lib
)
include_directories(
 include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
 add_executable(${PROJECT_NAME}_node src/pointer_game.cpp)

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}_node
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
 )

