cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# define this here or in .bashrc
# NOTE: youbot ubuntu is 32bit!!!!!!!!!!!!1
#set(OPENNI2_DIR /home/ariandy/src/OpenNI-Linux-x64-2.2/)
#set(NITE2_DIR /home/ariandy/src/NiTE-Linux-x64-2.2/)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# OpenNI2
link_directories(${OPENNI2_DIR}/Redist)
include_directories(${OPENNI2_DIR}/Include)

# NiTE2
link_directories(${NITE2_DIR}/Redist)
include_directories(${NITE2_DIR}/Include)

# OpenGL
include(FindGLUT)
include_directories(${GLUT_INCLUDE_DIR})

rosbuild_add_executable(user_selection 
	src/main.cpp
	src/userSelector.cpp
	src/userViewer.cpp
	)
target_link_libraries(user_selection OpenNI2 NiTE2 ${GLUT_LIBRARIES})
