cmake_minimum_required(VERSION 2.8.3)
project(mrta_vc)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	genmsg
	geometry_msgs
	message_generation
	nav_msgs
	roscpp
	std_msgs
	std_srvs
	)

find_package(Boost REQUIRED COMPONENTS
	system
	)

add_action_files(DIRECTORY action
	FILES
	Execute.action
	)

add_message_files(DIRECTORY msg
	FILES
	Agent.msg
	Allocation.msg
	Location.msg
	ManagerState.msg
	Place.msg
	Resource.msg
	Skill.msg
	Task.msg
	)
	
add_service_files(DIRECTORY srv
	FILES
	GenerateNewId.srv
	GetComputer.srv
	GetPerson.srv
	GetRobot.srv
	GetTask.srv
	GetUser.srv
	FinishAllocation.srv
	SetUser.srv
	ValidatePassword.srv
	)

generate_messages(
	DEPENDENCIES
	actionlib_msgs
	geometry_msgs
	mrta_vc
	std_msgs
	std_srvs
	)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mrta_vc_lib
	CATKIN_DEPENDS actionlib_msgs message_runtime
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	)

#add_library(mrta_vc_lib
#	src/agents/Agent.cpp
#	src/agents/Computer.cpp
#	src/agents/HierarchyLevels.cpp
#	src/agents/Person.cpp
#	src/agents/Robot.cpp
#	src/agents/User.cpp
#	src/places/Building.cpp
#	src/places/Campus.cpp
#	src/places/Desk.cpp
#	src/places/ExternalArea.cpp
#	src/places/Floor.cpp
#	src/places/Location.cpp
#	src/places/Office.cpp
#	src/places/Place.cpp
#	src/state_machine/AbstractState.cpp
#	src/state_machine/MachineController.cpp
#	src/state_machine/PersonVerificationState.cpp
#	src/state_machine/S0InitialState.cpp
#	src/state_machine/S1TaskVerificationState.cpp
#	src/state_machine/S2TaskVerificationState.cpp
#	src/state_machine/S3TaskVerificationState.cpp
#	src/state_machine/S4SenderVerificationState.cpp
#	src/state_machine/S5SenderVerificationState.cpp
#	src/state_machine/S6ReceiverVerificationState.cpp
#	src/state_machine/S7PriorityVerificationState.cpp
#	src/state_machine/S8DeadlineVerificationState.cpp
#	src/state_machine/S9FinalState.cpp
#	src/state_machine/SenderVerificationState.cpp
#	src/state_machine/TaskVerificationState.cpp
#	src/system/AllocationManager.cpp
#	src/system/DatabaseInterface.cpp
#	src/system/EntityTypes.cpp
#	src/tasks/Allocation.cpp
#	src/tasks/AllocationStates.cpp
#	src/tasks/Resource.cpp
#	src/tasks/Skill.cpp
#	src/tasks/SkillLevels.cpp
#	src/tasks/Task.cpp
#	src/tasks/TaskPriorities.cpp
#	src/tasks/TaskSatisfactions.cpp
#	src/utilities/MathManipulator.cpp
#	src/utilities/StringManipulator.cpp
#	src/utilities/TimeManipulator.cpp
#	)

#add_dependencies(mrta_vc_lib
#	${${PROJECT_NAME}_EXPORTED_TARGETS}
#	${catkin_EXPORTED_TARGETS}
#	)

add_executable(system_manager_node 
	src/system_manager_node.cpp
	include/mrta_vc/SystemManagerNode.h
	src/SystemManagerNode.cpp
	include/unifei/expertinos/mrta_vc/system/AllocationManager.h
	src/system/AllocationManager.cpp
	include/unifei/expertinos/mrta_vc/tasks/Allocation.h
	src/tasks/Allocation.cpp
	include/unifei/expertinos/mrta_vc/tasks/Task.h
	src/tasks/Task.cpp
	include/unifei/expertinos/mrta_vc/tasks/TaskPriorities.h
	src/tasks/TaskPriorities.cpp
	include/unifei/expertinos/mrta_vc/tasks/AllocationStates.h
	src/tasks/AllocationStates.cpp
	include/unifei/expertinos/mrta_vc/tasks/TaskSatisfactions.h
	src/tasks/TaskSatisfactions.cpp
	include/unifei/expertinos/mrta_vc/tasks/Skill.h
	src/tasks/Skill.cpp
	include/unifei/expertinos/mrta_vc/tasks/SkillLevels.h
	src/tasks/SkillLevels.cpp
	include/unifei/expertinos/mrta_vc/tasks/Resource.h
	src/tasks/Resource.cpp
	include/unifei/expertinos/mrta_vc/places/Location.h
	src/places/Location.cpp
	include/unifei/expertinos/mrta_vc/places/Place.h
	src/places/Place.cpp
	include/unifei/expertinos/mrta_vc/places/Campus.h
	src/places/Campus.cpp
	include/unifei/expertinos/mrta_vc/places/ExternalArea.h
	src/places/ExternalArea.cpp
	include/unifei/expertinos/mrta_vc/places/Building.h
	src/places/Building.cpp
	include/unifei/expertinos/mrta_vc/places/Floor.h
	src/places/Floor.cpp
	include/unifei/expertinos/mrta_vc/places/Office.h
	src/places/Office.cpp
	include/unifei/expertinos/mrta_vc/places/Desk.h
	src/places/Desk.cpp
	include/unifei/expertinos/mrta_vc/agents/Agent.h
	src/agents/Agent.cpp
	include/unifei/expertinos/mrta_vc/agents/Computer.h
	src/agents/Computer.cpp
	include/unifei/expertinos/mrta_vc/agents/Person.h
	src/agents/Person.cpp
	include/unifei/expertinos/mrta_vc/agents/HierarchyLevels.h
	src/agents/HierarchyLevels.cpp
	include/unifei/expertinos/mrta_vc/agents/Robot.h
	src/agents/Robot.cpp
	include/unifei/expertinos/mrta_vc/agents/User.h
	src/agents/User.cpp
	include/unifei/expertinos/utilities/StringManipulator.h
	src/utilities/StringManipulator.cpp
	include/unifei/expertinos/utilities/TimeManipulator.h
	src/utilities/TimeManipulator.cpp
	include/unifei/expertinos/utilities/MathManipulator.h
	src/utilities/MathManipulator.cpp
	)

add_dependencies(system_manager_node 
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	)

target_link_libraries(system_manager_node
	${catkin_LIBRARIES}
	)

add_executable(system_user_interface_node 
	src/system_user_interface_node.cpp
	include/mrta_vc/SystemUserInterfaceNode.h
	src/SystemUserInterfaceNode.cpp
	include/unifei/expertinos/mrta_vc/agents/User.h
	src/agents/User.cpp
	include/unifei/expertinos/mrta_vc/agents/Robot.h
	src/agents/Robot.cpp
	include/unifei/expertinos/mrta_vc/agents/Computer.h
	src/agents/Computer.cpp
	include/unifei/expertinos/mrta_vc/agents/Person.h
	src/agents/Person.cpp
	include/unifei/expertinos/mrta_vc/agents/HierarchyLevels.h
	src/agents/HierarchyLevels.cpp
	include/unifei/expertinos/mrta_vc/agents/Agent.h
	src/agents/Agent.cpp
	include/unifei/expertinos/mrta_vc/places/Location.h
	src/places/Location.cpp
	include/unifei/expertinos/mrta_vc/tasks/Allocation.h
	src/tasks/Allocation.cpp
	include/unifei/expertinos/mrta_vc/tasks/Task.h
	src/tasks/Task.cpp
	include/unifei/expertinos/mrta_vc/tasks/TaskPriorities.h
	src/tasks/TaskPriorities.cpp
	include/unifei/expertinos/mrta_vc/tasks/AllocationStates.h
	src/tasks/AllocationStates.cpp
	include/unifei/expertinos/mrta_vc/tasks/TaskSatisfactions.h
	src/tasks/TaskSatisfactions.cpp
	include/unifei/expertinos/mrta_vc/tasks/Skill.h
	src/tasks/Skill.cpp
	include/unifei/expertinos/mrta_vc/tasks/SkillLevels.h
	src/tasks/SkillLevels.cpp
	include/unifei/expertinos/mrta_vc/tasks/Resource.h
	src/tasks/Resource.cpp
	include/unifei/expertinos/utilities/StringManipulator.h
	src/utilities/StringManipulator.cpp
	include/unifei/expertinos/utilities/TimeManipulator.h
	src/utilities/TimeManipulator.cpp
	include/unifei/expertinos/utilities/MathManipulator.h
	src/utilities/MathManipulator.cpp
	)

add_dependencies(system_user_interface_node 
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	)

target_link_libraries(system_user_interface_node
	${catkin_LIBRARIES}
	)

add_executable(system_robot_interface_node 
	src/system_robot_interface_node.cpp
	include/mrta_vc/SystemRobotInterfaceNode.h
	src/SystemRobotInterfaceNode.cpp
	include/unifei/expertinos/mrta_vc/agents/Agent.h
	src/agents/Agent.cpp
	include/unifei/expertinos/mrta_vc/agents/Computer.h
	src/agents/Computer.cpp
	include/unifei/expertinos/mrta_vc/agents/Person.h
	src/agents/Person.cpp
	include/unifei/expertinos/mrta_vc/agents/HierarchyLevels.h
	src/agents/HierarchyLevels.cpp
	include/unifei/expertinos/mrta_vc/agents/Robot.h
	src/agents/Robot.cpp
	include/unifei/expertinos/mrta_vc/agents/User.h
	src/agents/User.cpp
	include/unifei/expertinos/mrta_vc/tasks/Allocation.h
	src/tasks/Allocation.cpp
	include/unifei/expertinos/mrta_vc/tasks/Task.h
	src/tasks/Task.cpp
	include/unifei/expertinos/mrta_vc/tasks/TaskPriorities.h
	src/tasks/TaskPriorities.cpp
	include/unifei/expertinos/mrta_vc/tasks/AllocationStates.h
	src/tasks/AllocationStates.cpp
	include/unifei/expertinos/mrta_vc/tasks/TaskSatisfactions.h
	src/tasks/TaskSatisfactions.cpp
	include/unifei/expertinos/mrta_vc/tasks/Skill.h
	src/tasks/Skill.cpp
	include/unifei/expertinos/mrta_vc/tasks/SkillLevels.h
	src/tasks/SkillLevels.cpp
	include/unifei/expertinos/mrta_vc/tasks/Resource.h
	src/tasks/Resource.cpp
	include/unifei/expertinos/mrta_vc/places/Location.h
	src/places/Location.cpp
	include/unifei/expertinos/utilities/StringManipulator.h
	src/utilities/StringManipulator.cpp
	include/unifei/expertinos/utilities/TimeManipulator.h
	src/utilities/TimeManipulator.cpp
	include/unifei/expertinos/utilities/MathManipulator.h
	src/utilities/MathManipulator.cpp
	)

add_dependencies(system_robot_interface_node 
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	)

target_link_libraries(system_robot_interface_node
	${catkin_LIBRARIES}
	)

add_executable(system_database_interface_node 
	src/system_database_interface_node.cpp
	include/mrta_vc/SystemDatabaseInterfaceNode.h
	src/SystemDatabaseInterfaceNode.cpp
	include/unifei/expertinos/mrta_vc/system/DatabaseInterface.h
	src/system/DatabaseInterface.cpp
	include/unifei/expertinos/mrta_vc/system/EntityTypes.h
	src/system/EntityTypes.cpp
	include/unifei/expertinos/mrta_vc/tasks/Allocation.h
	src/tasks/Allocation.cpp
	include/unifei/expertinos/mrta_vc/tasks/Task.h
	src/tasks/Task.cpp
	include/unifei/expertinos/mrta_vc/tasks/TaskPriorities.h
	src/tasks/TaskPriorities.cpp
	include/unifei/expertinos/mrta_vc/tasks/AllocationStates.h
	src/tasks/AllocationStates.cpp
	include/unifei/expertinos/mrta_vc/tasks/TaskSatisfactions.h
	src/tasks/TaskSatisfactions.cpp
	include/unifei/expertinos/mrta_vc/tasks/Skill.h
	src/tasks/Skill.cpp
	include/unifei/expertinos/mrta_vc/tasks/SkillLevels.h
	src/tasks/SkillLevels.cpp
	include/unifei/expertinos/mrta_vc/tasks/Resource.h
	src/tasks/Resource.cpp
	include/unifei/expertinos/mrta_vc/places/Location.h
	src/places/Location.cpp
	include/unifei/expertinos/mrta_vc/places/Place.h
	src/places/Place.cpp
	include/unifei/expertinos/mrta_vc/places/Campus.h
	src/places/Campus.cpp
	include/unifei/expertinos/mrta_vc/places/ExternalArea.h
	src/places/ExternalArea.cpp
	include/unifei/expertinos/mrta_vc/places/Building.h
	src/places/Building.cpp
	include/unifei/expertinos/mrta_vc/places/Floor.h
	src/places/Floor.cpp
	include/unifei/expertinos/mrta_vc/places/Office.h
	src/places/Office.cpp
	include/unifei/expertinos/mrta_vc/places/Desk.h
	src/places/Desk.cpp
	include/unifei/expertinos/mrta_vc/agents/Agent.h
	src/agents/Agent.cpp
	include/unifei/expertinos/mrta_vc/agents/Computer.h
	src/agents/Computer.cpp
	include/unifei/expertinos/mrta_vc/agents/Person.h
	src/agents/Person.cpp
	include/unifei/expertinos/mrta_vc/agents/HierarchyLevels.h
	src/agents/HierarchyLevels.cpp
	include/unifei/expertinos/mrta_vc/agents/Robot.h
	src/agents/Robot.cpp
	include/unifei/expertinos/mrta_vc/agents/User.h
	src/agents/User.cpp
	include/unifei/expertinos/utilities/StringManipulator.h
	src/utilities/StringManipulator.cpp
	include/unifei/expertinos/utilities/TimeManipulator.h
	src/utilities/TimeManipulator.cpp
	include/unifei/expertinos/utilities/MathManipulator.h
	src/utilities/MathManipulator.cpp
	)

add_dependencies(system_database_interface_node 
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	)

target_link_libraries(system_database_interface_node
	${catkin_LIBRARIES}
	)

add_executable(task_builder_node 
	src/task_builder_node.cpp
	include/mrta_vc/TaskBuilderNode.h
	src/TaskBuilderNode.cpp
	include/unifei/expertinos/mrta_vc/tasks/Task.h
	src/tasks/Task.cpp
	include/unifei/expertinos/mrta_vc/tasks/TaskPriorities.h
	src/tasks/TaskPriorities.cpp
	include/unifei/expertinos/mrta_vc/tasks/TaskSatisfactions.h
	src/tasks/TaskSatisfactions.cpp
	include/unifei/expertinos/mrta_vc/tasks/Skill.h
	src/tasks/Skill.cpp
	include/unifei/expertinos/mrta_vc/tasks/SkillLevels.h
	src/tasks/SkillLevels.cpp
	include/unifei/expertinos/mrta_vc/tasks/Resource.h
	src/tasks/Resource.cpp
	include/unifei/expertinos/mrta_vc/places/Location.h
	src/places/Location.cpp
	include/unifei/expertinos/mrta_vc/agents/Agent.h
	src/agents/Agent.cpp
	include/unifei/expertinos/mrta_vc/agents/Person.h
	src/agents/Person.cpp
	include/unifei/expertinos/mrta_vc/agents/Computer.h
	src/agents/Computer.cpp
	include/unifei/expertinos/mrta_vc/agents/HierarchyLevels.h
	src/agents/HierarchyLevels.cpp
	include/unifei/expertinos/mrta_vc/agents/User.h
	src/agents/User.cpp
	include/mrta_vc/state_machine/MachineController.h
	src/state_machine/MachineController.cpp
	include/mrta_vc/state_machine/AbstractState.h
	src/state_machine/AbstractState.cpp
	include/mrta_vc/state_machine/TaskVerificationState.h
	src/state_machine/TaskVerificationState.cpp
	include/mrta_vc/state_machine/PersonVerificationState.h
	src/state_machine/PersonVerificationState.cpp
	include/mrta_vc/state_machine/SenderVerificationState.h
	src/state_machine/SenderVerificationState.cpp
	include/mrta_vc/state_machine/S0InitialState.h
	src/state_machine/S0InitialState.cpp
	include/mrta_vc/state_machine/S1TaskVerificationState.h
	src/state_machine/S1TaskVerificationState.cpp
	include/mrta_vc/state_machine/S2TaskVerificationState.h
	src/state_machine/S2TaskVerificationState.cpp
	include/mrta_vc/state_machine/S3TaskVerificationState.h
	src/state_machine/S3TaskVerificationState.cpp
	include/mrta_vc/state_machine/S4SenderVerificationState.h
	src/state_machine/S4SenderVerificationState.cpp
	include/mrta_vc/state_machine/S5SenderVerificationState.h
	src/state_machine/S5SenderVerificationState.cpp
	include/mrta_vc/state_machine/S6ReceiverVerificationState.h
	src/state_machine/S6ReceiverVerificationState.cpp
	include/mrta_vc/state_machine/S7PriorityVerificationState.h
	src/state_machine/S7PriorityVerificationState.cpp
	include/mrta_vc/state_machine/S8DeadlineVerificationState.h
	src/state_machine/S8DeadlineVerificationState.cpp
	include/mrta_vc/state_machine/S9FinalState.h
	src/state_machine/S9FinalState.cpp
	include/unifei/expertinos/utilities/StringManipulator.h
	src/utilities/StringManipulator.cpp
	include/unifei/expertinos/utilities/TimeManipulator.h
	src/utilities/TimeManipulator.cpp
	include/unifei/expertinos/utilities/MathManipulator.h
	src/utilities/MathManipulator.cpp
	include/unifei/expertinos/mrta_vc/system/EntityTypes.h
	src/system/EntityTypes.cpp
	)

add_dependencies(task_builder_node 
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	)

target_link_libraries(task_builder_node
	${catkin_LIBRARIES}
	)

add_executable(speech_analyser_node 
	src/speech_analyser_node.cpp
	include/mrta_vc/SpeechAnalyserNode.h
	src/SpeechAnalyserNode.cpp
	include/unifei/expertinos/utilities/StringManipulator.h
	src/utilities/StringManipulator.cpp
	include/unifei/expertinos/utilities/TimeManipulator.h
	src/utilities/TimeManipulator.cpp
	include/unifei/expertinos/utilities/MathManipulator.h
	src/utilities/MathManipulator.cpp
	)

add_dependencies(speech_analyser_node
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	)

target_link_libraries(speech_analyser_node
	${catkin_LIBRARIES}
	)
	
# Qt Creator Stuff:

file(GLOB children ${CMAKE_SOURCE_DIR}/*)
foreach(child ${children})
	if(IS_DIRECTORY ${child} AND NOT ${child} STREQUAL ${PROJECT_SOURCE_DIR}/.git)
		file(GLOB_RECURSE dir_files "${child}/*")
		list(APPEND extra_files ${dir_files})
	endif()
endforeach()
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${extra_files})

