cmake_minimum_required(VERSION 2.8.3)
project(likelihood_grid)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  hark_msgs
  tf
  autonomy_human
  roscpp
  dynamic_reconfigure
  rospy
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/Test.cfg
)

# make sure configure headers are built before any node using them
find_package(Boost REQUIRED COMPONENTS
system
)

set(CMAKE_BUILD_TYPE Debug)

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

add_executable(likelihood_grid_node  src/likelihood_grid_node.cpp src/likelihood_grid.cpp src/grid.cpp )
add_executable(leg_grid_node         src/leg_grid_node.cpp src/cleggrid.cpp src/grid.cpp )
add_executable(sound_grid_node         src/sound_grid_node.cpp src/csoundgrid.cpp src/grid.cpp )

add_executable(vision_grid_node         src/vision_grid_node.cpp src/cvisiongrid.cpp src/grid.cpp )

add_executable(human_grid_node  src/human_grid_node.cpp src/chumangrid.cpp src/grid.cpp)
add_executable(test_node src/test.cpp )
add_executable(data_node src/data_extraction.cpp)
add_executable(distance_data_node src/distance_data.cpp)
add_dependencies(test_node ${PROJECT_NAME}_gencfg)


## Specify libraries to link a library or executable target against
target_link_libraries(likelihood_grid_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(leg_grid_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(sound_grid_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(vision_grid_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(human_grid_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(test_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(data_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(distance_data_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)



