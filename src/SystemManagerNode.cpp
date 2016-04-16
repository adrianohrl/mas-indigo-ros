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
void mrta_vc::SystemManagerNode::robotBeaconCallback(const mrta_vc::Agent::ConstPtr& beacon_msg)
{
	ROS_INFO("[ROBOT] Passei aki!!!");
}

/**
 * 
 */
void mrta_vc::SystemManagerNode::userBeaconCallback(const mrta_vc::Agent::ConstPtr& beacon_msg)
{
	ROS_INFO("[USER] Passei aki!!!");
}
