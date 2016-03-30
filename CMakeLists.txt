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
	Location.msg
	Resource.msg
	Skill.msg
	Task.msg
	TaskAllocation.msg
	)

generate_messages(
	DEPENDENCIES
	geometry_msgs
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
	src/tasks/Skill.cpp
	src/tasks/Resource.cpp
	)

add_dependencies(system_manager_node 
	mrta_vc_gen_cpp
	${catkin_EXPORTED_TARGETS}
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
	${catkin_EXPORTED_TARGETS}
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
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(system_robot_interface_node
	${catkin_LIBRARIES}
	)
