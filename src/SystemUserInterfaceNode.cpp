/**
 *  SystemUserInterfaceNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemUserInterfaceNode.h"

/**
 * Constructor
 */
mrta_vc::SystemUserInterfaceNode::SystemUserInterfaceNode(ros::NodeHandle nh) :
	nh_(nh)
{
}

/**
 * Destructor
 */
mrta_vc::SystemUserInterfaceNode::~SystemUserInterfaceNode()
{
}

/**
 * 
 */
void mrta_vc::SystemUserInterfaceNode::spin() 
{
	ROS_INFO("System User Interface Node is up and running!!!");
	ros::Rate loop_rate(10.0);
	while (nh_.ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
