project(RobotSimulator)
cmake_minimum_required(VERSION 2.8)

# remember to set VREP_HOME variable
# eg. export VREP_HOME=/home/user/vrep/...
# in terminal or in .bashrc file
if(NOT DEFINED ENV{VREP_HOME})
    message(FATAL_ERROR "Define VREP_HOME environment variable!")
endif()

include_directories(remoteApi)
include_directories($ENV{VREP_HOME}/programming/include)

add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D__linux)

add_executable(robot main.cpp remoteApi/extApi.c remoteApi/extApiPlatform.c)
target_link_libraries(robot pthread)
