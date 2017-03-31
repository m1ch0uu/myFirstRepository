cmake_minimum_required(VERSION 2.8.3)
project(collision_avoidance)

include_directories(SYSTEM)

find_package(catkin REQUIRED COMPONENTS
	pcl_ros
 	roscpp
 	joy
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  pcl_ros
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

add_executable(collision_avoidance src/coll_avoid.cpp)
target_link_libraries(collision_avoidance ${catkin_LIBRARIES})
target_link_libraries(collision_avoidance m)