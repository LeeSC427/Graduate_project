cmake_minimum_required(VERSION 3.0.2)
project(lidar_pointcloud)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lidar_pointcloud
  CATKIN_DEPENDS rosconsole roscpp sensor_msgs cv_bridge
  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
 add_executable(${PROJECT_NAME}_node src/lidar_pointcloud.cpp)

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
  )

