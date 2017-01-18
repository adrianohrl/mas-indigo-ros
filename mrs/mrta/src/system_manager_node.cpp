/**
 *  This source file implements the main function that calls the System Manager
 *node controller.
 *
 *  Version: 1.4.0
 *  Created n: 17/08/2016
 *  Modified on: 04/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta/system_manager_node.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "system_manager_node");
	ros::NodeHandle nh;
  mrta::SystemManagerNode node(&nh);
	node.spin();
	return 0;
}
