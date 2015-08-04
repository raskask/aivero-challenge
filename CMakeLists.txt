cmake_minimum_required(VERSION 2.8.3)
project(programming_test)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
add_message_files(
	DIRECTORY
	msg
	FILES
	Solution.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
	scripts/publisher.py
	scripts/verify.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

