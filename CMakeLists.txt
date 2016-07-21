project(RobotSimulator)
cmake_minimum_required(VERSION 2.8)

# remember to set VREP_HOME variable
# eg. export VREP_HOME=/home/user/vrep/...
# in terminal or in .bashrc file
if(NOT DEFINED ENV{VREP_HOME})
    
    message(WARNING "Define VREP_HOME environment variable! Only tests will be built")

else()

    include_directories(remoteApi)
    include_directories($ENV{VREP_HOME}/programming/include)

    add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D__linux)

    add_executable(robot main.cpp remoteApi/extApi.c remoteApi/extApiPlatform.c)
    target_link_libraries(robot pthread)
    
endif()

SET(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS_DEBUG "-g -O0 -fprofile-arcs -ftest-coverage")

#
# Testing
#
find_package (Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})
add_definitions (-DBOOST_TEST_DYN_LINK)

add_executable (orientation_test orientation_test.cpp)
target_link_libraries (orientation_test ${Boost_LIBRARIES})

enable_testing()
add_test (NAME orientation_test COMMAND orientation_test)

#
# Code coverage
#
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(CMAKE_COMPILER_IS_GNUCXX)
    include(CodeCoverage)
    setup_target_for_coverage(test_coverage orientation_test coverage)
endif()
