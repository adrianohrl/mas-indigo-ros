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
	logged_robots_update_timer_ = nh_.createTimer(ros::Duration(ROBOT_BEACON_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::loggedRobotsUpdateTimerCallback, this);
	logged_users_update_timer_ = nh_.createTimer(ros::Duration(USER_BEACON_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::loggedUsersUpdateTimerCallback, this);
	robot_beacon_sub_ = nh_.subscribe("/robot_beacon", 100, &mrta_vc::SystemManagerNode::robotBeaconCallback, this);
	user_beacon_sub_ = nh_.subscribe("/user_beacon", 100, &mrta_vc::SystemManagerNode::userBeaconCallback, this);
}

/**
 * Destructor
 */
mrta_vc::SystemManagerNode::~SystemManagerNode()
{
	robot_beacon_sub_.shutdown();
	user_beacon_sub_.shutdown();
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
void mrta_vc::SystemManagerNode::robotBeaconCallback(const mrta_vc::Agent::ConstPtr& robot_msg)
{
	unifei::expertinos::mrta_vc::agents::Robot robot(robot_msg);
	robot.setLastBeaconTimestamp();
	unifei::expertinos::mrta_vc::system::AllocationManager::addRobot(robot);
}

/**
 * 
 */
void mrta_vc::SystemManagerNode::userBeaconCallback(const mrta_vc::Agent::ConstPtr& user_msg)
{
	unifei::expertinos::mrta_vc::agents::VoiceCommander user(user_msg);
	user.setLastBeaconTimestamp();
	unifei::expertinos::mrta_vc::system::AllocationManager::addUser(user);
}

/**
 *
 */
void mrta_vc::SystemManagerNode::loggedRobotsUpdateTimerCallback(const ros::TimerEvent& event) 
{
	unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedRobots();
	ROS_INFO("[ROBOTS] Total number of logged robots: %d", (int) getLoggedRobots().size());
}

/**
 *
 */
void mrta_vc::SystemManagerNode::loggedUsersUpdateTimerCallback(const ros::TimerEvent& event) 
{
	unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedUsers();
	ROS_INFO("[USERS] Total number of logged users: %d", (int) getLoggedUsers().size());
}
