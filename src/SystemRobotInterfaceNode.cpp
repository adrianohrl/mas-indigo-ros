/**
 *  SystemRobotInterfaceNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemRobotInterfaceNode.h"

/**
 * Constructor
 */
mrta_vc::SystemRobotInterfaceNode::SystemRobotInterfaceNode(ros::NodeHandle nh) : unifei::expertinos::mrta_vc::agents::Robot(), nh_(nh)
{
	beacon_timer_ = nh_.createTimer(ros::Duration(ROBOT_BEACON_INTERVAL_DURATION), &mrta_vc::SystemRobotInterfaceNode::beaconTimerCallback, this);
	beacon_pub_ = nh_.advertise<mrta_vc::Agent>("/robot_beacon", 1);
}

/**
 * Destructor
 */
mrta_vc::SystemRobotInterfaceNode::~SystemRobotInterfaceNode()
{
	beacon_pub_.shutdown();
}

/**
 * 
 */
void mrta_vc::SystemRobotInterfaceNode::spin() 
{
	ROS_INFO("System Robot Interface Node is up and running!!!");
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
void mrta_vc::SystemRobotInterfaceNode::beaconTimerCallback(const ros::TimerEvent& event) 
{
	beacon_pub_.publish(unifei::expertinos::mrta_vc::agents::Robot::toMsg());
}
