cmake_minimum_required(VERSION 2.8.3)
project(autonomy_human)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

find_package(OpenCV)

add_message_files(
   FILES
   human.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES autonomy_human
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp std_msgs
  DEPENDS system_lib opencv message_runtime
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(autonomy_human src/autonomy_human.cpp)
add_dependencies(autonomy_human ${autonomy_human_EXPORTED_TARGETS})

 target_link_libraries(autonomy_human
   ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
 )
