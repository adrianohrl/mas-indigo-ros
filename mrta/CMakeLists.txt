cmake_minimum_required(VERSION 2.8.3)

project(mrta)

find_package(catkin REQUIRED COMPONENTS actionlib mrta_vc roscpp std_msgs	std_srvs)

find_package(mas_lib REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mas_lib
	CATKIN_DEPENDS actionlib	mrta_vc roscpp	std_msgs std_srvs
)

include_directories(
	include
	${mas_lib_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_executable(system_manager_node src/system_manager_node.cpp src/SystemManagerNode.cpp)
add_executable(system_user_interface_node src/system_user_interface_node.cpp src/SystemUserInterfaceNode.cpp)
add_executable(system_robot_interface_node  src/system_robot_interface_node.cpp src/SystemRobotInterfaceNode.cpp)
add_executable(system_database_interface_node src/system_database_interface_node.cpp src/SystemDatabaseInterfaceNode.cpp)
add_executable(task_builder_node src/task_builder_node.cpp src/TaskBuilderNode.cpp)
add_executable(speech_analyser_node src/speech_analyser_node.cpp	src/SpeechAnalyserNode.cpp)
	
add_dependencies(system_manager_node ${catkin_EXPORTED_TARGETS})
add_dependencies(system_user_interface_node ${catkin_EXPORTED_TARGETS})
add_dependencies(system_robot_interface_node ${catkin_EXPORTED_TARGETS})
add_dependencies(system_database_interface_node ${catkin_EXPORTED_TARGETS})
add_dependencies(task_builder_node ${catkin_EXPORTED_TARGETS})
add_dependencies(speech_analyser_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(system_manager_node ${catkin_LIBRARIES} ${mas_lib_LIBRARIES})
target_link_libraries(system_user_interface_node ${catkin_LIBRARIES} ${mas_lib_LIBRARIES})
target_link_libraries(system_robot_interface_node ${catkin_LIBRARIES} ${mas_lib_LIBRARIES})
target_link_libraries(system_database_interface_node ${catkin_LIBRARIES} ${mas_lib_LIBRARIES})
target_link_libraries(task_builder_node ${catkin_LIBRARIES} ${mas_lib_LIBRARIES})
target_link_libraries(speech_analyser_node ${catkin_LIBRARIES} ${mas_lib_LIBRARIES})

