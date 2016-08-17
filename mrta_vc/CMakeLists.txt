cmake_minimum_required(VERSION 2.8.3)

project(mrta_vc)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs genmsg geometry_msgs message_generation nav_msgs std_msgs)

add_message_files(
	DIRECTORY msg
	FILES Agent.msg
		Allocation.msg
		Location.msg
		ManagerState.msg
		Place.msg
		Resource.msg
		Skill.msg
		Task.msg
)
	
add_service_files(
	DIRECTORY srv
	FILES GenerateNewId.srv
		GetComputer.srv
		GetPerson.srv
		GetRobot.srv
		GetTask.srv
		GetUser.srv
		FinishAllocation.srv
		SetUser.srv
		ValidatePassword.srv
)

add_action_files(
	DIRECTORY action
	FILES Execute.action
)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs mrta_vc std_msgs)

catkin_package(CATKIN_DEPENDS	actionlib_msgs	geometry_msgs message_runtime	nav_msgs std_msgs)
