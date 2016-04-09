cmake_minimum_required(VERSION 2.8.3)
project(mrta_vc)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	message_generation
	roscpp
	std_msgs
	)

add_message_files(
	FILES
	Agent.msg
	Allocation.msg
	Location.msg
	Place.msg
	Resource.msg
	Skill.msg
	Task.msg
	)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	mrta_vc
	std_msgs
	)

catkin_package(
	CATKIN_DEPENDS 
	message_runtime
	)

include_directories(
	include 
	${catkin_INCLUDE_DIRS}
	)

add_executable(system_manager_node 
	src/system_manager_node.cpp
	src/SystemManagerNode.cpp
	src/tasks/Allocation.cpp
	src/tasks/Task.cpp
	src/tasks/TaskPriorities.cpp
	src/tasks/TaskStates.cpp
	src/tasks/TaskSatisfactions.cpp
	src/tasks/Skill.cpp
	src/tasks/SkillLevels.cpp
	src/tasks/Resource.cpp
	src/places/Location.cpp
	src/places/Place.cpp
	src/places/Campus.cpp
	src/places/ExternalArea.cpp
	src/places/Building.cpp
	src/places/Floor.cpp
	src/places/Office.cpp
	src/places/Desk.cpp
	src/agents/Agent.cpp
	src/agents/Computer.cpp
	src/agents/Person.cpp
	src/agents/Robot.cpp
	src/agents/VoiceCommander.cpp
	)

add_dependencies(system_manager_node 
	mrta_vc_gen_cpp
	${mrta_vc_EXPORTED_TARGETS}
	)

target_link_libraries(system_manager_node
	${catkin_LIBRARIES}
	)

add_executable(system_user_interface_node 
	src/system_user_interface_node.cpp
	src/SystemUserInterfaceNode.cpp
	)

add_dependencies(system_user_interface_node 
	mrta_vc_gen_cpp
	${mrta_vc_EXPORTED_TARGETS}
	)

target_link_libraries(system_user_interface_node
	${catkin_LIBRARIES}
	)

add_executable(system_robot_interface_node 
	src/system_robot_interface_node.cpp
	src/SystemRobotInterfaceNode.cpp
	)

add_dependencies(system_robot_interface_node 
	mrta_vc_gen_cpp
	${mrta_vc_EXPORTED_TARGETS}
	)

target_link_libraries(system_robot_interface_node
	${catkin_LIBRARIES}
	)

add_executable(system_database_interface_node 
	src/system_database_interface_node.cpp
	src/SystemDatabaseInterfaceNode.cpp
	)

add_dependencies(system_database_interface_node 
	mrta_vc_gen_cpp
	${mrta_vc_EXPORTED_TARGETS}
	)

target_link_libraries(system_database_interface_node
	${catkin_LIBRARIES}
	)

add_executable(task_builder_node 
	src/task_builder_node.cpp
	src/TaskBuilderNode.cpp
	)

add_dependencies(task_builder_node 
	mrta_vc_gen_cpp
	${mrta_vc_EXPORTED_TARGETS}
	)

target_link_libraries(task_builder_node
	${catkin_LIBRARIES}
	)

add_executable(speech_analyser_node 
	src/speech_analyser_node.cpp
	src/SpeechAnalyserNode.cpp
	)

add_dependencies(speech_analyser_node 
	mrta_vc_gen_cpp
	${mrta_vc_EXPORTED_TARGETS}
	)

target_link_libraries(speech_analyser_node
	${catkin_LIBRARIES}
	)
