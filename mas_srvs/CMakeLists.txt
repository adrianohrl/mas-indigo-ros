cmake_minimum_required(VERSION 2.8.3)

project(mas_srvs)

find_package(catkin REQUIRED COMPONENTS mas_msgs message_generation std_msgs)

add_service_files(
	DIRECTORY srv
	FILES GenerateNewId.srv
		GetComputer.srv
		GetPerson.srv
		GetRobot.srv
		GetTask.srv
		GetUser.srv
                FinishAllocation.srv
		ValidatePassword.srv
)

generate_messages(DEPENDENCIES mas_msgs std_msgs)

catkin_package(CATKIN_DEPENDS mas_msgs message_runtime std_msgs)

