/**
 *  SystemManagerNode.cpp
 *
 *  Version: 1.2.2
 *  Created on: 26/03/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta/SystemManagerNode.h"

using typename mas::agents::Robot;
using typename mas::agents::User;
using typename mas::database::TaskAllocator;
using typename mas::tasks::Allocation;
using typename mas::tasks::Skill;
using typename mas::tasks::SkillLevels;
using typename mas::tasks::Task;

namespace mrta
{

	/**
	 * Constructor
	 */
	SystemManagerNode::SystemManagerNode(ros::NodeHandle nh) : nh_(nh), execute_action_cli_("/execute_task", true)
	{
		robots_sub_ = nh_.subscribe("/robots", 100, &SystemManagerNode::robotsCallback, this);
		tasks_sub_ = nh_.subscribe("/tasks", 100, &SystemManagerNode::tasksCallback, this);
		users_sub_ = nh_.subscribe("/users", 100, &SystemManagerNode::usersCallback, this);;
		manager_state_pub_ = nh.advertise<mas_msgs::ManagerState>("/manager_state", 1);\
		robots_timer_ = nh_.createTimer(ros::Duration(.75 * ROBOT_BEACON_INTERVAL_DURATION), &SystemManagerNode::robotsTimerCallback, this);
		tasks_timer_ = nh_.createTimer(ros::Duration(TASK_INTERVAL_DURATION), &SystemManagerNode::tasksTimerCallback, this);
		users_timer_ = nh_.createTimer(ros::Duration(.75 * USER_BEACON_INTERVAL_DURATION), &SystemManagerNode::usersTimerCallback, this);
	}

	/**
	 * Destructor
	 */
	SystemManagerNode::~SystemManagerNode()
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
	void SystemManagerNode::spin() 
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
	void SystemManagerNode::robotsCallback(const mas_msgs::Agent::ConstPtr& robot_msg)
	{
		Robot robot(robot_msg);
		robot.setLastBeaconTimestamp();
		TaskAllocator::add(robot);
	}

	/**
	 * This callback receives new valid tasks.
	 */
	void SystemManagerNode::tasksCallback(const mas_msgs::Task::ConstPtr& task_msg)
	{
		Task task(task_msg);
		TaskAllocator::add(task);

									/************** Showing new task info 17/08/2016*****/
									//task = TaskAllocator::getUnallocatedTasks().top();
									ROS_WARN("[MANAGER] ------- NEW TASK -------");
									ROS_ERROR("[MANAGER] id: %d, name: %s", task.getId(), task.getName().c_str());
									ROS_INFO("[MANAGER] skills:");
									std::vector<Skill> skills = task.getDesiredSkills();
									for (int i = 0; i < skills.size(); i++)
									{
										Skill skill(skills.at(i));
										ROS_INFO("[MANAGER] %s %s", SkillLevels::toString(skill.getLevel()).c_str(),
														 skill.getResource().getName().c_str());
									}
									ROS_ERROR("[MANAGER] deadline: %s", utilities::TimeManipulator::toString(task.getDeadline()).c_str());
									/****************************************************/
	}

	/**
	 * This callback receives new user beacon signals so that system can keep track of who is still logged in.
	 */
	void SystemManagerNode::usersCallback(const mas_msgs::Agent::ConstPtr& user_msg)
	{
		User user(user_msg);
		user.setLastBeaconTimestamp();
		TaskAllocator::add(user);
	}

	/**
	 * This callback triggers the robots logon verification. That is, ...
	 */
	void SystemManagerNode::robotsTimerCallback(const ros::TimerEvent& event)
	{
		TaskAllocator::updateLoggedRobots();
	}

	/**
	 * This callback triggers the (new) allocation verification. That is, ...
	 */
	void SystemManagerNode::tasksTimerCallback(const ros::TimerEvent& event)
	{
		TaskAllocator::updateUnallocatedTasks();
	}

	/**
	 * This callback triggers the users logon verification. That is, ...
	 */
	void SystemManagerNode::usersTimerCallback(const ros::TimerEvent& event)
	{
		TaskAllocator::updateLoggedUsers();
	}

	/**
	 *
	 */
	void SystemManagerNode::dispatch(Allocation allocation)
	{
		ROS_ERROR_COND(TaskAllocator::areThereAnyAvailableRobots() &&
									 !execute_action_cli_.isServerConnected(), "There is no execute server connected!!!");
		if (!allocation.wasDispatched())
		{
			ROS_DEBUG("[MANAGER] Sending %s allocation!!!", allocation.getTask().getName().c_str());
			allocation.dispatch();
			if (allocation.wasDispatched())
			{
				mas_actions::ExecuteGoal goal;
				goal.allocation = allocation.toMsg();
				execute_action_cli_.sendGoal(goal, boost::bind(&SystemManagerNode::allocationResultCallback, this, _1, _2),
																					 boost::bind(&SystemManagerNode::allocationActiveCallback, this),
																					 boost::bind(&SystemManagerNode::allocationFeedbackCallback, this, _1));
				TaskAllocator::dispatch(allocation);
			}
		}
	}

	/**
	 * FOR TESTS AND VALIDATIONS
	 */
	void SystemManagerNode::managerStatePublish()
	{
		mas_msgs::ManagerState manager_state_msg;
		manager_state_msg.number_of_unallocated_tasks = TaskAllocator::getUnallocatedTasks().size();
		manager_state_msg.number_of_allocated_tasks = TaskAllocator::getAllocatedTasks().size();
		manager_state_msg.number_of_available_robots = TaskAllocator::getAvailableRobots().size();
		manager_state_msg.number_of_busy_robots = TaskAllocator::getBusyRobots().size();
		manager_state_msg.number_of_logged_users = TaskAllocator::getLoggedUsers().size();
		manager_state_msg.number_of_allocations = TaskAllocator::getAllocations().size();
		manager_state_pub_.publish(manager_state_msg);
	}

	/**
	 *
	 */
	void SystemManagerNode::allocationActiveCallback()
	{
		ROS_INFO("Goal just went active!!!");
	}

	/**
	 *
	 */
	void SystemManagerNode::allocationFeedbackCallback(const mas_actions::ExecuteFeedback::ConstPtr& feedback)
	{
		Allocation allocation(feedback->allocation);
		//ROS_INFO("%s allocation state: %s", allocation.getTask().getName().c_str(), AllocationStates::toString(allocation.getState()).c_str());
		TaskAllocator::updateAllocations(allocation);
	}

	/**
	 *
	 */
	void SystemManagerNode::allocationResultCallback(const actionlib::SimpleClientGoalState& state, const mas_actions::ExecuteResult::ConstPtr& result)
	{
		Allocation allocation(result->allocation);
		ROS_INFO("%s allocation finished in state [%s]", allocation.getTask().getName().c_str(), state.toString().c_str());
		TaskAllocator::updateAllocations(allocation);
	}
	
}
