cmake_minimum_required(VERSION 2.8.3)

project(mrta)

find_package(catkin REQUIRED COMPONENTS actionlib roscpp std_msgs std_srvs)

find_package(mas REQUIRED COMPONENTS mas_actions mas_msgs mas_srvs)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
	INCLUDE_DIRS include
        LIBRARIES mas
        CATKIN_DEPENDS actionlib mas_actions mas_msgs mas_srvs roscpp std_msgs std_srvs
)

include_directories(
	include
        ${mas_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

set(H_FILES
       include/mrta/system_manager_node.h
)

set(CPP_FILES
       src/mrta/system_manager_node.cpp
)

add_executable(system_manager_node src/system_manager_node.cpp ${CPP_FILES})
add_dependencies(system_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(system_manager_node ${catkin_LIBRARIES} ${mas_LIBRARIES})

