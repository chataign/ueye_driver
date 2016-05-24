cmake_minimum_required(VERSION 2.8.3)
project(ueye_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ascamm_quadrotor_viz
#  CATKIN_DEPENDS enu nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} )
add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

file( GLOB SOURCES src/*.cpp )
add_executable( ${PROJECT_NAME} ${SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ueye_api )
