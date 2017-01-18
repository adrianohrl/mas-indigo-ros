cmake_minimum_required(VERSION 2.8.3)

project(mas)

find_package(catkin REQUIRED COMPONENTS geometry_msgs mas_msgs mas_srvs nav_msgs roscpp std_msgs std_srvs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS geometry_msgs mas_msgs mas_srvs nav_msgs roscpp std_msgs std_srvs)

include_directories(include ${catkin_INCLUDE_DIRS})

set(H_FILES
	 include/mas/agents/agent.h
	 include/mas/agents/computer.h
	 include/mas/agents/hierarchy_levels.h
	 include/mas/agents/person.h
	 include/mas/agents/robot.h
	 include/mas/agents/user.h
	 include/mas/database/database_interface.h
	 include/mas/database/entity_types.h
	 include/mas/database/task_allocator.h
	 include/mas/places/building.h
	 include/mas/places/campus.h
	 include/mas/places/desk.h
	 include/mas/places/external_area.h
	 include/mas/places/floor.h
	 include/mas/places/location.h
	 include/mas/places/office.h
	 include/mas/places/place.h
	 include/mas/tasks/allocation.h
	 include/mas/tasks/allocation_satisfactions.h
	 include/mas/tasks/allocation__states.h
	 include/mas/tasks/resource.h
	 include/mas/tasks/skill.h
	 include/mas/tasks/skill_levels.h
	 include/mas/tasks/task.h
         include/mas/tasks/task_priorities.h
	 include/mrs/robots/level.h
	 include/mrs/robots/level_converter.h
	 include/mrs/robots/capability.h
	 include/mrs/robots/resource.h
	 include/utilities/binary_expression_tree/logical/and.h
	 include/utilities/binary_expression_tree/logical/logical_operator.h
	 include/utilities/binary_expression_tree/logical/not.h
	 include/utilities/binary_expression_tree/logical/or.h
	 include/utilities/binary_expression_tree/logical/xor.h
	 include/utilities/binary_expression_tree/binary_expression_tree.h
	 include/utilities/binary_expression_tree/node.h
	 include/utilities/binary_expression_tree/operand.h
	 include/utilities/binary_expression_tree/operator.h
	 include/utilities/capability_expression_evaluator/capability_expression_evaluator.h
	 include/utilities/capability_expression_evaluator/capability_operand.h
	 include/utilities/capability_expression_evaluator/capability_parser.h
	 include/utilities/enum_converter.h
         include/utilities/exception.h
	 include/utilities/math_manipulator.h
         include/utilities/ros_message.h
         include/utilities/ros_node.h
	 include/utilities/string_manipulator.h
	 include/utilities/time_manipulator.h
)

set(CPP_FILES
	 src/mas/agents/agent.cpp
	 src/mas/agents/computer.cpp
	 src/mas/agents/hierarchy_levels.cpp
	 src/mas/agents/person.cpp
	 src/mas/agents/robot.cpp
	 src/mas/agents/user.cpp
	 src/mas/database/database_interface.cpp
	 src/mas/database/entity_types.cpp
	 src/mas/database/task_allocator.cpp
	 src/mas/places/building.cpp
	 src/mas/places/campus.cpp
	 src/mas/places/desk.cpp
	 src/mas/places/external_area.cpp
	 src/mas/places/floor.cpp
	 src/mas/places/location.cpp
	 src/mas/places/office.cpp
	 src/mas/places/place.cpp
	 src/mas/tasks/allocation.cpp
	 src/mas/tasks/allocation_satisfactions.cpp
	 src/mas/tasks/allocation_states.cpp
	 src/mas/tasks/resource.cpp
	 src/mas/tasks/skill.cpp
	 src/mas/tasks/skill_levels.cpp
         src/mas/tasks/task.cpp
	 src/mas/tasks/task_priorities.cpp
	 src/mrs/robots/levels/level_converter.cpp
	 src/mrs/robots/capability.cpp
	 src/mrs/robots/resource.cpp
	 src/utilities/capability_expression_evaluator/capability_expression_evaluator.cpp
	 src/utilities/capability_expression_evaluator/capability_operand.cpp
	 src/utilities/capability_expression_evaluator/capability_parser.cpp
         src/utilities/exception.cpp
	 src/utilities/math_manipulator.cpp
         src/utilities/ros_node.cpp
	 src/utilities/string_manipulator.cpp
	 src/utilities/time_manipulator.cpp
)

add_library(${PROJECT_NAME} ${CPP_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
