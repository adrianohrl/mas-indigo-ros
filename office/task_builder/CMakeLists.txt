cmake_minimum_required(VERSION 2.8.3)

project(task_builder)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs)

find_package(mas REQUIRED mas_actions mas_msgs mas_srvs)

catkin_package(
	INCLUDE_DIRS include
        LIBRARIES mas
	CATKIN_DEPENDS mas_msgs mas_srvs roscpp std_msgs std_srvs
)

include_directories(
	include
        ${mas_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

set(H_FILES
         include/task_builder/abstract_state.h
         include/task_builder/machine_controller.h
         include/task_builder/person_verification_state.h
         include/task_builder/s0_initial_state.h
         include/task_builder/s1_task_verification_state.h
         include/task_builder/s2_task_verification_state.h
         include/task_builder/s3_task_verification_state.h
         include/task_builder/s4_sender_verification_state.h
         include/task_builder/s5_sender_verification_state.h
         include/task_builder/s6_receiver_verification_state.h
         include/task_builder/s7_priority_verification_state.h
         include/task_builder/s8_deadline_verification_state.h
         include/task_builder/s9_final_state.h
         include/task_builder/sender_verification_state.h
         include/task_builder/task_verification_state.h
)

set(CPP_FILES
         src/task_builder/abstract_state.cpp
         src/task_builder/machine_controller.cpp
         src/task_builder/person_verification_state.cpp
         src/task_builder/s0_initial_state.cpp
         src/task_builder/s1_task_verification_state.cpp
         src/task_builder/s2_task_verification_state.cpp
         src/task_builder/s3_task_verification_state.cpp
         src/task_builder/s4_sender_verification_state.cpp
         src/task_builder/s5_sender_verification_state.cpp
         src/task_builder/s6_receiver_verification_state.cpp
         src/task_builder/s7_priority_verification_state.cpp
         src/task_builder/s8_deadline_verification_state.cpp
         src/task_builder/s9_final_state.cpp
         src/task_builder/sender_verification_state.cpp
         src/task_builder/task_builder_node.cpp
         src/task_builder/task_verification_state.cpp
)

add_executable(task_builder_node src/task_builder_node.cpp ${CPP_FILES})
add_dependencies(task_builder_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_builder_node ${catkin_LIBRARIES} ${mas_LIBRARIES})

