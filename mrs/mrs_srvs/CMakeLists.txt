cmake_minimum_required(VERSION 2.8.3)

project(mrs_srvs)

find_package(catkin REQUIRED 
	COMPONENTS mrs_msgs
		message_generation
		std_msgs
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

generate_messages(DEPENDENCIES mrs_msgs std_msgs)

catkin_package(CATKIN_DEPENDS mrs_msgs message_runtime std_msgs)

