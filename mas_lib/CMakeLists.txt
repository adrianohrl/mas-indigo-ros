cmake_minimum_required(VERSION 2.8.3)

project(mas_lib)

find_package(catkin REQUIRED COMPONENTS geometry_msgs mas_msgs mas_srvs nav_msgs roscpp std_msgs std_srvs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS geometry_msgs mas_msgs mas_srvs nav_msgs roscpp std_msgs std_srvs)

include_directories(include ${catkin_INCLUDE_DIRS})
	
file(STRINGS HEADER_FILES header-files)
file(STRINGS SOURCE_FILES source-files)

string(REGEX REPLACE "\n" " " ${header-files} ${header-files})
string(REGEX REPLACE "\n" " " ${source-files} ${source-files})

add_library(${PROJECT_NAME} ${source-files})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
