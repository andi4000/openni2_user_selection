cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(OPENNI2_DIR /home/ariandy/src/OpenNI-Linux-x64-2.2/)
set(NITE2_DIR /home/ariandy/src/NiTE-Linux-x64-2.2/)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

link_directories(${OPENNI2_DIR}/Redist)
include_directories(${OPENNI2_DIR}/Include)

link_directories(${NITE2_DIR}/Redist)
include_directories(${NITE2_DIR}/Include)

rosbuild_add_executable(user_selection 
	src/main.cpp
	src/userSelector.cpp
	)
target_link_libraries(user_selection OpenNI2 NiTE2)
