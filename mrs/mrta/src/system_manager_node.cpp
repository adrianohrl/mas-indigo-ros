/**
 *  main.cpp
 *
 *  Version: 1.2.4
 *  Created n: 17/08/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta/SystemManagerNode.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "system_manager_node");
	ros::NodeHandle nh;
  	mrta::SystemManagerNode node(nh);
	node.spin();
	return 0;
}
