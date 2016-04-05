/**
 *  SystemDatabaseInterfaceNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Christiano Henrique Rezende (c.h.rezende@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemDatabaseInterfaceNode.h"

/**
 * Constructor
 */
mrta_vc::SystemDatabaseInterfaceNode::SystemDatabaseInterfaceNode(ros::NodeHandle nh) :
	nh_(nh)
{
}

/**
 * Destructor
 */
mrta_vc::SystemDatabaseInterfaceNode::~SystemDatabaseInterfaceNode()
{
}

/**
 * 
 */
void mrta_vc::SystemDatabaseInterfaceNode::spin() 
{
	ROS_INFO("System Database Interface Node is up and running!!!");
	ros::Rate loop_rate(10.0);
	while (nh_.ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
