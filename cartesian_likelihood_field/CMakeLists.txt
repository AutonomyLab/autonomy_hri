cmake_minimum_required(VERSION 2.8.3)
project(cartesian_likelihood_field)
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


add_executable(cartesian_likelihood_field src/cartesian_likelihood_field.cpp src/cartesian_grid.cpp src/cartesian_grid_interface.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cartesian_likelihood_field
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)
