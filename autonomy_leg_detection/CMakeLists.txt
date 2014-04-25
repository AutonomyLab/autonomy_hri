cmake_minimum_required(VERSION 2.8.3)
project(autonomy_leg_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  miarn_ros
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
 # LIBRARIES autonomy_legdetection
  CATKIN_DEPENDS cv_bridge geometry_msgs miarn_ros roscpp std_msgs
  DEPENDS system_lib opencv
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(leg_detection src/leg_detection.cpp)

target_link_libraries(
    leg_detection
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

