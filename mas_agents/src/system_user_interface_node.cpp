/**
 *  main.cpp
 *
 *  Version: 1.2.4
 *  Created on: 26/03/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas_agents/SystemUserInterfaceNode.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "system_user_interface_node");
	ros::NodeHandle nh;
	mas_agents::SystemUserInterfaceNode node(nh);
	node.spin();
	return 0;
}
