cmake_minimum_required(VERSION 2.8.3)

project(mas_database)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

find_package(mas REQUIRED COMPONENTS mas_actions mas_msgs mas_srvs)

catkin_package(
	INCLUDE_DIRS include
        LIBRARIES mas
	CATKIN_DEPENDS mas_msgs mas_srvs roscpp std_msgs
)

include_directories(
	include
        ${mas_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

set(H_FILES
        include/mas_database/system_database_interface_node.h
)

set(CPP_FILES
        src/mas_database/system_database_interface_node.cpp
)

add_executable(system_database_interface_node src/system_database_interface_node.cpp ${CPP_FILES})
add_dependencies(system_database_interface_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(system_database_interface_node ${catkin_LIBRARIES} ${mas_LIBRARIES})

