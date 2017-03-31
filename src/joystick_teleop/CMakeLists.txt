cmake_minimum_required(VERSION 2.8.3)
project(joystick_teleop)

include_directories(SYSTEM)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  joy
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  joy
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


link_directories("/opt/ros/fuerte/lib")
link_directories("/opt/ros/groovy/lib")
link_directories("/opt/ros/hydro/lib")
link_directories("/opt/ros/indigo/lib")
link_directories("/opt/ros/jade/lib")
link_directories("/opt/ros/kinetic/lib")

add_executable(joystick_teleop src/joystick_teleop_node.cpp)
target_link_libraries(joystick_teleop ${catkin_LIBRARIES})
