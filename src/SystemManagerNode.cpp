/**
 *  SystemManagerNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemManagerNode.h"

/**
 * Constructor
 */
mrta_vc::SystemManagerNode::SystemManagerNode(ros::NodeHandle nh) : nh_(nh), execute_action_cli_("/execute_task", true)
{
	robots_sub_ = nh_.subscribe("/robots", 100, &mrta_vc::SystemManagerNode::robotsCallback, this);
	tasks_sub_ = nh_.subscribe("/tasks", 100, &mrta_vc::SystemManagerNode::tasksCallback, this);
	users_sub_ = nh_.subscribe("/users", 100, &mrta_vc::SystemManagerNode::usersCallback, this);;
	manager_state_pub_ = nh.advertise<mrta_vc::ManagerState>("/manager_state", 1);\
	robots_timer_ = nh_.createTimer(ros::Duration(.75 * ROBOT_BEACON_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::robotsTimerCallback, this);
	tasks_timer_ = nh_.createTimer(ros::Duration(TASK_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::tasksTimerCallback, this);
	users_timer_ = nh_.createTimer(ros::Duration(.75 * USER_BEACON_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::usersTimerCallback, this);
}

/**
 * Destructor
 */
mrta_vc::SystemManagerNode::~SystemManagerNode()
{
	execute_action_cli_.stopTrackingGoal();
	robots_timer_.stop();
	tasks_timer_.stop();
	users_timer_.stop();
	robots_sub_.shutdown();
	tasks_sub_.shutdown();
	users_sub_.shutdown();
	manager_state_pub_.shutdown();
}

/**
 * While spining, the manager state is published
 */
void mrta_vc::SystemManagerNode::spin() 
{
	ROS_INFO("System Manager Node is up and running!!!");
	ros::Rate loop_rate(10.0);
	while (nh_.ok())
	{
		managerStatePublish(); // for tests and validations
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/**
 * This callback eceives new robot beacon signals in order to keep track of which robots are still logged in
 * the system.
 */
void mrta_vc::SystemManagerNode::robotsCallback(const mrta_vc::Agent::ConstPtr& robot_msg)
{
	unifei::expertinos::mrta_vc::agents::Robot robot(robot_msg);
	robot.setLastBeaconTimestamp();
	unifei::expertinos::mrta_vc::system::AllocationManager::add(robot);
}

/**
 * This callback receives new valid tasks.
 */
void mrta_vc::SystemManagerNode::tasksCallback(const mrta_vc::Task::ConstPtr& task_msg)
{
	unifei::expertinos::mrta_vc::tasks::Task task(task_msg);
	unifei::expertinos::mrta_vc::system::AllocationManager::add(task);

								/************** Showing new task info **************/
								//task = unifei::expertinos::mrta_vc::system::AllocationManager::getUnallocatedTasks().top();
								ROS_WARN("[MANAGER] ------- NEW TASK -------");
								ROS_ERROR("[MANAGER] id: %d, name: %s", task.getId(), task.getName().c_str());
								ROS_INFO("[MANAGER] skills:");
								std::vector<unifei::expertinos::mrta_vc::tasks::Skill> skills = task.getDesiredSkills();
								for (int i = 0; i < skills.size(); i++)
								{
									unifei::expertinos::mrta_vc::tasks::Skill skill(skills.at(i));
									ROS_INFO("[MANAGER] %s %s", unifei::expertinos::mrta_vc::tasks::SkillLevels::toString(skill.getLevel()).c_str(),
													 skill.getResource().getName().c_str());
								}
								ROS_ERROR("[MANAGER] deadline: %s", unifei::expertinos::utilities::TimeManipulator::toString(task.getDeadline()).c_str());
								/****************************************************/
}

/**
 * This callback receives new user beacon signals so that system can keep track of who is still logged in.
 */
void mrta_vc::SystemManagerNode::usersCallback(const mrta_vc::Agent::ConstPtr& user_msg)
{
	unifei::expertinos::mrta_vc::agents::User user(user_msg);
	user.setLastBeaconTimestamp();
	unifei::expertinos::mrta_vc::system::AllocationManager::add(user);
}

/**
 * This callback triggers the robots logon verification. That is, ...
 */
void mrta_vc::SystemManagerNode::robotsTimerCallback(const ros::TimerEvent& event)
{
	unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedRobots();
}

/**
 * This callback triggers the (new) allocation verification. That is, ...
 */
void mrta_vc::SystemManagerNode::tasksTimerCallback(const ros::TimerEvent& event)
{
	unifei::expertinos::mrta_vc::system::AllocationManager::updateUnallocatedTasks();
}

/**
 * This callback triggers the users logon verification. That is, ...
 */
void mrta_vc::SystemManagerNode::usersTimerCallback(const ros::TimerEvent& event)
{
	unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedUsers();
}

/**
 *
 */
void mrta_vc::SystemManagerNode::dispatch(unifei::expertinos::mrta_vc::tasks::Allocation allocation)
{
	ROS_ERROR_COND(unifei::expertinos::mrta_vc::system::AllocationManager::areThereAnyAvailableRobots() &&
								 !execute_action_cli_.isServerConnected(), "There is no execute server connected!!!");
	if (!allocation.wasDispatched())
	{
		ROS_DEBUG("[MANAGER] Sending %s allocation!!!", allocation.getTask().getName().c_str());
		allocation.dispatch();
		if (allocation.wasDispatched())
		{
			mrta_vc::ExecuteGoal goal;
			goal.allocation = allocation.toMsg();
			execute_action_cli_.sendGoal(goal, boost::bind(&mrta_vc::SystemManagerNode::allocationResultCallback, this, _1, _2),
																				 boost::bind(&mrta_vc::SystemManagerNode::allocationActiveCallback, this),
																				 boost::bind(&mrta_vc::SystemManagerNode::allocationFeedbackCallback, this, _1));
			unifei::expertinos::mrta_vc::system::AllocationManager::dispatch(allocation);
		}
	}
}

/**
 * FOR TESTS AND VALIDATIONS
 */
void mrta_vc::SystemManagerNode::managerStatePublish()
{
	mrta_vc::ManagerState manager_state_msg;
	manager_state_msg.number_of_unallocated_tasks = unifei::expertinos::mrta_vc::system::AllocationManager::getUnallocatedTasks().size();
	manager_state_msg.number_of_allocated_tasks = unifei::expertinos::mrta_vc::system::AllocationManager::getAllocatedTasks().size();
	manager_state_msg.number_of_available_robots = unifei::expertinos::mrta_vc::system::AllocationManager::getAvailableRobots().size();
	manager_state_msg.number_of_busy_robots = unifei::expertinos::mrta_vc::system::AllocationManager::getBusyRobots().size();
	manager_state_msg.number_of_logged_users = unifei::expertinos::mrta_vc::system::AllocationManager::getLoggedUsers().size();
	manager_state_msg.number_of_allocations = unifei::expertinos::mrta_vc::system::AllocationManager::getAllocations().size();
	manager_state_pub_.publish(manager_state_msg);
}

/**
 *
 */
void mrta_vc::SystemManagerNode::allocationActiveCallback()
{
	ROS_INFO("Goal just went active!!!");
}

/**
 *
 */
void mrta_vc::SystemManagerNode::allocationFeedbackCallback(const mrta_vc::ExecuteFeedback::ConstPtr& feedback)
{
	unifei::expertinos::mrta_vc::tasks::Allocation allocation(feedback->allocation);
	//ROS_INFO("%s allocation state: %s", allocation.getTask().getName().c_str(), unifei::expertinos::mrta_vc::tasks::AllocationStates::toString(allocation.getState()).c_str());
	unifei::expertinos::mrta_vc::system::AllocationManager::updateAllocations(allocation);
}

/**
 *
 */
void mrta_vc::SystemManagerNode::allocationResultCallback(const actionlib::SimpleClientGoalState& state, const mrta_vc::ExecuteResult::ConstPtr& result)
{
	unifei::expertinos::mrta_vc::tasks::Allocation allocation(result->allocation);
	ROS_INFO("%s allocation finished in state [%s]", allocation.getTask().getName().c_str(), state.toString().c_str());
	unifei::expertinos::mrta_vc::system::AllocationManager::updateAllocations(allocation);
}
