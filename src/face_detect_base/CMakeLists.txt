cmake_minimum_required(VERSION 2.8.3)
project(face_detect_base)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
   FILES
   perso_roi_type.msg
 )

generate_messages(
	DEPENDENCIES 
	sensor_msgs   
	std_msgs
)

catkin_package(
	LIBRARIES 
	CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


link_directories("/opt/ros/fuerte/lib")
link_directories("/opt/ros/groovy/lib")
link_directories("/opt/ros/hydro/lib")
link_directories("/opt/ros/indigo/lib")
link_directories("/opt/ros/jade/lib")
link_directories("/opt/ros/kinetic/lib")