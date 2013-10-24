cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(move_base src/move_base.cpp)
#rosbuild_add_executable(see_sonar src/see_sonar.cpp)
#rosbuild_add_executable(see_laser src/see-laser.cpp)
#rosbuild_add_executable(findClosestPoint2Laser src/findClosestPoint2Laser.cpp)
#rosbuild_add_executable(legDetection src/legDetection.cpp)
#rosbuild_add_executable(LegTalk src/LegTalk.cpp)
#rosbuild_add_executable(justTalk src/justTalk.cpp)
#rosbuild_add_executable(findLegs src/findLegs.cpp)
#rosbuild_add_executable(soundWalk src/soundWalk.cpp)
#rosbuild_add_executable(velSound src/velSound.cpp)
#rosbuild_add_executable(kinectTilt src/kinectTilt.cpp)
rosbuild_add_executable(GestureRec src/GestureRec.cpp)
rosbuild_add_executable(pioneerHRI src/pioneerHRI.cpp)
#rosbuild_add_executable(newGestureRec src/newGestureRec.cpp)
