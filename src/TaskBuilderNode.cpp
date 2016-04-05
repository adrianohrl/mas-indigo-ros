/**
 *  TaskBuilderNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/TaskBuilderNode.h"

/**
 * Constructor
 */
mrta_vc::TaskBuilderNode::TaskBuilderNode(ros::NodeHandle nh) :
	nh_(nh)
{
}

/**
 * Destructor
 */
mrta_vc::TaskBuilderNode::~TaskBuilderNode()
{
}

/**
 * 
 */
void mrta_vc::TaskBuilderNode::spin() 
{
	ROS_INFO("System Database Interface Node is up and running!!!");
	ros::Rate loop_rate(10.0);
	while (nh_.ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
