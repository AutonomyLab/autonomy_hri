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
  roslint
)

cmake_policy(SET CMP0046 OLD)

# Force using custom openCV
#set(CUSTOM_OPENCV_DIR "/home/autolab/opencv/opencv-3.1.0/install")
#find_package(OpenCV 3 REQUIRED PATHS ${CUSTOM_OPENCV_DIR} NO_DEFAULT_PATH)

# Use system's openCV 3.x
find_package(OpenCV 3 REQUIRED )

message(STATUS "OpenCV Includes: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV Libraries: ${OpenCV_LIBRARIES}")
message(STATUS "OpenCV Libs: ${OpenCV_LIBS}")
message(STATUS "OpenCV Lib dir: ${OpenCV_LIB_DIR}")


# or
# find_package(OpenCV)

add_message_files(
   FILES
   human.msg
   raw_detections.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES autonomy_human
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp std_msgs message_runtime
  DEPENDS opencv3
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

roslint_cpp()
