/**
 *  main.cpp
 *
 *  Version: 1.2.4
 *  Created on: 26/03/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrs_agents/SystemRobotInterfaceNode.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "system_robot_interface_node");
	ros::NodeHandle nh;
	mrs_agents::SystemRobotInterfaceNode node(nh);
	node.spin();
	return 0;
}
