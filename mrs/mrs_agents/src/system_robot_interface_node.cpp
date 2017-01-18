/**
 *  This source file implements the main function that calls the System Robot
 *Interface node controller.
 *
 *  Version: 1.4.0
 *  Created on: 26/03/2016
 *  Modified on: 06/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrs_agents/system_robot_interface_node.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "system_robot_interface_node");
	ros::NodeHandle nh;
  mrs_agents::SystemRobotInterfaceNode node(&nh);
	node.spin();
	return 0;
}
