/**
 *  main.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: 01/04/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/TaskBuilderNode.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "task_builder_node");
	ros::NodeHandle nh;
	mrta_vc::TaskBuilderNode node(nh);
	node.spin();
	return 0;
}
