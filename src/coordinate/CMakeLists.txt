cmake_minimum_required(VERSION 3.0.2)
project(coordinate)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
)

find_package(OpenCV REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES coordinate
  CATKIN_DEPENDS cv_bridge roscpp
  DEPENDS system_lib
)

include_directories(
 include
 ${OpenCV_INCLUDE_DIRS} 
 ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node src/coordinate.cpp)

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )


