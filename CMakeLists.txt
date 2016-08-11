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

set(libmas_VERSION 1.0.0)
set(libmas_DIR ~/libmas-${libmas_VERSION}/release)
find_package(libmas ${libmas_VERSION} REQUIRED EXACT)

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
	CATKIN_DEPENDS actionlib_msgs message_runtime
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${libmas_INCLUDE_DIRS}
)

link_directories(
	${libmas_INCLUDE_DIRS}
)

add_executable(system_manager_node 
	src/system_manager_node.cpp
	include/mrta_vc/SystemManagerNode.h
	src/SystemManagerNode.cpp
)

add_dependencies(system_manager_node 
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(system_manager_node
	${catkin_LIBRARIES}
	${libmas_LIBRARIES}
)

add_executable(system_user_interface_node 
	src/system_user_interface_node.cpp
	include/mrta_vc/SystemUserInterfaceNode.h
	src/SystemUserInterfaceNode.cpp
)

add_dependencies(system_user_interface_node 
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(system_user_interface_node
	${catkin_LIBRARIES}
	${libmas_LIBRARIES}
)

add_executable(system_robot_interface_node 
	src/system_robot_interface_node.cpp
	include/mrta_vc/SystemRobotInterfaceNode.h
	src/SystemRobotInterfaceNode.cpp
)

add_dependencies(system_robot_interface_node 
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(system_robot_interface_node
	${catkin_LIBRARIES}
	${libmas_LIBRARIES}
)

add_executable(system_database_interface_node 
	src/system_database_interface_node.cpp
	include/mrta_vc/SystemDatabaseInterfaceNode.h
	src/SystemDatabaseInterfaceNode.cpp
)

add_dependencies(system_database_interface_node 
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(system_database_interface_node
	${catkin_LIBRARIES}
	${libmas_LIBRARIES}
)

add_executable(task_builder_node 
	src/task_builder_node.cpp
	include/mrta_vc/TaskBuilderNode.h
	src/TaskBuilderNode.cpp
)

add_dependencies(task_builder_node 
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(task_builder_node
	${catkin_LIBRARIES}
	${libmas_LIBRARIES}
)

add_executable(speech_analyser_node 
	src/speech_analyser_node.cpp
	include/mrta_vc/SpeechAnalyserNode.h
	src/SpeechAnalyserNode.cpp
)

add_dependencies(speech_analyser_node
	${PROJECT_NAME}_gen_cpp
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(speech_analyser_node
	${catkin_LIBRARIES}
	${libmas_LIBRARIES}
)

########################
### Qt Creator Stuff ###
########################

file(GLOB children ${CMAKE_SOURCE_DIR}/*)
foreach(child ${children})
	if(IS_DIRECTORY ${child} AND NOT ${child} STREQUAL ${PROJECT_SOURCE_DIR}/.git)
		file(GLOB_RECURSE dir_files "${child}/*")
		list(APPEND extra_files ${dir_files})
	endif()
endforeach()
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${extra_files})

