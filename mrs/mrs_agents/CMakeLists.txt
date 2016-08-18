cmake_minimum_required(VERSION 2.8.3)

project(mrs_agents)

find_package(catkin REQUIRED COMPONENTS actionlib roscpp std_msgs std_srvs)

find_package(mas_lib REQUIRED COMPONENTS mas_actions mas_msgs mas_srvs)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mas_lib
	CATKIN_DEPENDS actionlib mas_actions mas_msgs mas_srvs roscpp std_msgs std_srvs
)

include_directories(
	include
	${mas_lib_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_executable(system_robot_interface_node  src/system_robot_interface_node.cpp src/SystemRobotInterfaceNode.cpp)
add_dependencies(system_robot_interface_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(system_robot_interface_node ${catkin_LIBRARIES} ${mas_lib_LIBRARIES})

