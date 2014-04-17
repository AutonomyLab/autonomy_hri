cmake_minimum_required(VERSION 2.8.3)
project(likelihood_field)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  tf
  autonomy_human
)
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs
# )
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES likelihood_field
  CATKIN_DEPENDS geometry_msgs sensor_msgs tf autonomy_human
  DEPENDS system_lib opencv
)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)

## Declare a cpp library
# add_library(likelihood_field
#   src/${PROJECT_NAME}/likelihood_field.cpp
# )

add_executable(likelihood_field src/likelihood_field.cpp src/lkgrid.cpp src/lkgrid_interface.cpp )

## Specify libraries to link a library or executable target against
target_link_libraries(likelihood_field
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
