/**
 *  main.cpp
 *
 *  Version: 1.2.4
 *  Created on: 01/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas_task_builder/TaskBuilderNode.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "task_builder_node");
	ros::NodeHandle nh;
	mas_task_builder::TaskBuilderNode node(nh);
	node.spin();
	return 0;
}
