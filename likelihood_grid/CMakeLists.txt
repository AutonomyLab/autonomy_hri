cmake_minimum_required(VERSION 2.8.3)
project(likelihood_grid)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  hark_msgs
  tf
  autonomy_human
)

find_package(Boost REQUIRED COMPONENTS
system
)

set(ROS_BUILD_TYPE Debug)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS hark_msgs geometry_msgs sensor_msgs tf autonomy_human
  DEPENDS system_lib opencv
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_executable(likelihood_grid src/likelihood_grid_node.cpp src/likelihood_grid.cpp src/grid.cpp src/leg_grid.cpp src/kalmanfilter.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(likelihood_grid
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)
