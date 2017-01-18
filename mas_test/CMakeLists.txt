cmake_minimum_required(VERSION 2.8.3)

project(mas_test)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs)

find_package(mas REQUIRED COMPONENTS mas_actions mas_msgs mas_srvs)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mas
	CATKIN_DEPENDS mas_actions mas_msgs mas_srvs roscpp std_msgs std_srvs
)

include_directories(
	include
        ${mas_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(capability_parser_test src/capability_parser_test.cpp)
add_dependencies(capability_parser_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(capability_parser_test ${catkin_LIBRARIES} ${mas_LIBRARIES})

