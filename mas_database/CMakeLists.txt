cmake_minimum_required(VERSION 2.8.3)

project(mas_database)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

find_package(mas_lib REQUIRED COMPONENTS mas_actions mas_msgs mas_srvs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mas_lib
	CATKIN_DEPENDS mas_msgs mas_srvs roscpp std_msgs
)

include_directories(
	include
	${mas_lib_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_executable(system_database_interface_node src/system_database_interface_node.cpp src/SystemDatabaseInterfaceNode.cpp)
add_dependencies(system_database_interface_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(system_database_interface_node ${catkin_LIBRARIES} ${mas_lib_LIBRARIES})

