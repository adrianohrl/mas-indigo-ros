/**
 *  main.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemManagerNode.h"
#include "br/edu/unifei/expertinos/mrta_vc/tasks/Skill.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "system_manager_node");
	ros::NodeHandle nh;
	br::edu::unifei::expertinos::mrta_vc::tasks::Resource resource(0, "adriano", "");
	br::edu::unifei::expertinos::mrta_vc::tasks::Skill skill(resource);
	mrta_vc::SystemManagerNode node(nh);
	node.spin();
	return 0;
}
