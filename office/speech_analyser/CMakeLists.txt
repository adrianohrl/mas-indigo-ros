cmake_minimum_required(VERSION 2.8.3)

project(speech_analyser)

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

set(H_FILES
        include/speech_analyser/speech_analyser_node.h
)

set(CPP_FILES
        src/speech_analyser/speech_analyser_node.cpp
)

add_executable(speech_analyser_node src/speech_analyser_node.cpp ${CPP_FILES})
add_dependencies(speech_analyser_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(speech_analyser_node ${catkin_LIBRARIES} ${mas_LIBRARIES})

