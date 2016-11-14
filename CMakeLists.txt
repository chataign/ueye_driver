cmake_minimum_required(VERSION 2.8.3)
project(ueye_driver)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11 )

if( NOT COMPILER_SUPPORTS_CXX11 )
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror")
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_library(UEYE_LIBRARY ueye_api)

if( NOT UEYE_LIBRARY )
  message(FATAL_ERROR "libueye_api was not found in system's library path")
endif()

message(STATUS "ueye_api library: ${UEYE_LIBRARY}")
message(STATUS "ueye_api include: ${UEYE_INCLUDE_DIR}")

find_package(catkin REQUIRED COMPONENTS
  roscpp image_transport nodelet sensor_msgs camera_calibration_parsers dynamic_reconfigure
)

generate_dynamic_reconfigure_options( ${PROJECT_NAME}.cfg )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} ${UEYE_INCLUDE_DIR} )

file( GLOB SOURCES src/*.cpp )
add_library( ${PROJECT_NAME} ${SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${UEYE_LIBRARY} )

add_dependencies( ${PROJECT_NAME} ${PROJECT_NAME}_gencfg )
