cmake_minimum_required(VERSION 2.8.3)

project(mrs_actions)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs genmsg message_generation mrs_msgs std_msgs)

add_action_files(
	DIRECTORY action
	FILES Execute.action
)

generate_messages(DEPENDENCIES actionlib_msgs mrs_msgs std_msgs)

catkin_package(CATKIN_DEPENDS	actionlib_msgs message_runtime mrs_msgs std_msgs)
