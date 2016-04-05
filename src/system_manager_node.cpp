/**
 *  main.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: 26/03/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemManagerNode.h"
#include "unifei/expertinos/mrta_vc/tasks/Skill.h"
#include "unifei/expertinos/mrta_vc/places/Desk.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "system_manager_node");
	ros::NodeHandle nh;
	unifei::expertinos::mrta_vc::tasks::Resource resource(0, "adriano", "");
	unifei::expertinos::mrta_vc::tasks::Skill skill(resource);
	unifei::expertinos::mrta_vc::places::Campus campus(1, "Sede da UNIFEI", geometry_msgs::Polygon(), 0.0, 0.0);
	unifei::expertinos::mrta_vc::places::Building building(2, "IEE", campus, geometry_msgs::Polygon(), 0.0, 0.0);
	unifei::expertinos::mrta_vc::places::Floor floor(3, "1st", building, geometry_msgs::Polygon(), 0.0, 0.0);
	unifei::expertinos::mrta_vc::places::Office office(4, "LRO", floor, geometry_msgs::Polygon(), 0.0, 0.0);
	unifei::expertinos::mrta_vc::places::Desk desk(5, "Adriano's Desk", office, geometry_msgs::Polygon(), 0.0, 0.0);
	mrta_vc::SystemManagerNode node(nh);
	node.spin();
	return 0;
}
