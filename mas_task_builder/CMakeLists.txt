cmake_minimum_required(VERSION 2.8.3)

project(mas_task_builder)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs)

find_package(mas_lib REQUIRED mas_actions mas_msgs mas_srvs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mas_lib
	CATKIN_DEPENDS mas_msgs mas_srvs roscpp std_msgs std_srvs
)

include_directories(
	include
	${mas_lib_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(task_builder_node src/task_builder_node.cpp src/TaskBuilderNode.cpp)
add_dependencies(task_builder_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_builder_node ${catkin_LIBRARIES} ${mas_lib_LIBRARIES})

