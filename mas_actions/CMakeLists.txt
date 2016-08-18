cmake_minimum_required(VERSION 2.8.3)

project(mas_actions)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs genmsg mas_msgs message_generation std_msgs)

add_action_files(
	DIRECTORY action
	FILES Execute.action
)

generate_messages(DEPENDENCIES actionlib_msgs mas_msgs std_msgs)

catkin_package(CATKIN_DEPENDS	actionlib_msgs mas_msgs message_runtime std_msgs)
