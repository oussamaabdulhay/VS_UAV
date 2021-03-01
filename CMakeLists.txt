cmake_minimum_required(VERSION 2.8.3)
project(vs_uav)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  tf2_ros
  tf2
  geometry_msgs
  tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)


catkin_package(
INCLUDE_DIRS include LIBRARIES rosopencv_t CATKIN_DEPENDS hear_ros_bridge cv_bridge image_transport roscpp sensor_msgs tf2_geometry_msgs DEPENDS )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(target_position_node src/target_position_node.cpp)
add_executable(ball_detector_node src/detect_ball_node.cpp)
add_library(ball_detector src/BallDetectorRgb.cpp)

target_link_libraries(ball_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(ball_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ball_detector)
target_link_libraries(target_position_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


