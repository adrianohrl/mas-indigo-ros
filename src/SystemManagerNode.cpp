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
mrta_vc::SystemManagerNode::SystemManagerNode(ros::NodeHandle nh) : nh_(nh)
{
  robots_sub_ = nh_.subscribe("/robots", 100, &mrta_vc::SystemManagerNode::robotsCallback, this);
  tasks_sub_ = nh_.subscribe("/tasks", 100, &mrta_vc::SystemManagerNode::tasksCallback, this);
  users_sub_ = nh_.subscribe("/users", 100, &mrta_vc::SystemManagerNode::usersCallback, this);
  robots_timer_ = nh_.createTimer(ros::Duration(.75 * ROBOT_BEACON_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::robotsTimerCallback, this);
  tasks_timer_ = nh_.createTimer(ros::Duration(TASK_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::tasksTimerCallback, this);
  users_timer_ = nh_.createTimer(ros::Duration(.75 * USER_BEACON_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::usersTimerCallback, this);
}

/**
 * Destructor
 */
mrta_vc::SystemManagerNode::~SystemManagerNode()
{
  robots_timer_.stop();
  tasks_timer_.stop();
  users_timer_.stop();
  robots_sub_.shutdown();
  tasks_sub_.shutdown();
  users_sub_.shutdown();
}

/**
 * 
 */
void mrta_vc::SystemManagerNode::spin() 
{
	ROS_INFO("System Manager Node is up and running!!!");
	ros::Rate loop_rate(10.0);
	while (nh_.ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/**
 * 
 */
void mrta_vc::SystemManagerNode::robotsCallback(const mrta_vc::Agent::ConstPtr& robot_msg)
{
	unifei::expertinos::mrta_vc::agents::Robot robot(robot_msg);
	robot.setLastBeaconTimestamp();
	unifei::expertinos::mrta_vc::system::AllocationManager::add(robot);
}

/**
 *
 */
void mrta_vc::SystemManagerNode::tasksCallback(const mrta_vc::Task::ConstPtr& task_msg)
{
  ROS_WARN("IMPLEMENTAR mrta_vc::SystemManagerNode::tasksCallback!!!");
}

/**
 *
 */
void mrta_vc::SystemManagerNode::usersCallback(const mrta_vc::Agent::ConstPtr& user_msg)
{
  unifei::expertinos::mrta_vc::agents::VoiceCommander user(user_msg);
  user.setLastBeaconTimestamp();
  unifei::expertinos::mrta_vc::system::AllocationManager::add(user);
}

/**
 *
 */
void mrta_vc::SystemManagerNode::robotsTimerCallback(const ros::TimerEvent& event)
{
  unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedRobots();
}

/**
 *
 */
void mrta_vc::SystemManagerNode::tasksTimerCallback(const ros::TimerEvent& event)
{
  ROS_WARN("IMPLEMENTAR mrta_vc::SystemManagerNode::tasksTimerCallback!!!");
}

/**
 *
 */
void mrta_vc::SystemManagerNode::usersTimerCallback(const ros::TimerEvent& event)
{
  unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedUsers();
}
