cmake_minimum_required(VERSION 2.8.3)

project(mas_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation nav_msgs std_msgs)

add_message_files(
	DIRECTORY msg
	FILES Agent.msg
		Allocation.msg
		Computer.msg
		Location.msg
		ManagerState.msg
		Person.msg
		Place.msg
		Resource.msg
		Robot.msg
		Skill.msg
		Task.msg
		User.msg
)

generate_messages(DEPENDENCIES geometry_msgs mas_msgs nav_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs std_msgs)

