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
#include "unifei/expertinos/mrta_vc/tasks/Task.h"
#include "unifei/expertinos/mrta_vc/places/Desk.h"
#include "unifei/expertinos/mrta_vc/agents/Robot.h"
#include "unifei/expertinos/mrta_vc/agents/VoiceCommander.h"

using namespace unifei::expertinos::mrta_vc;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "system_manager_node");

	/*** Task Related Classes ***/
	tasks::Resource resource(0, "adriano", "");
	tasks::Skill skill(resource);
	tasks::Task task();

	/*** Place Related Classes ***/
	places::Campus campus(1, "Sede da UNIFEI", geometry_msgs::Polygon(), 0.0, 0.0);
	places::Building building(2, "IEE", campus, geometry_msgs::Polygon(), 0.0, 0.0);
	places::Floor floor(3, "1st", building, geometry_msgs::Polygon(), 0.0, 0.0);
	places::Office office(4, "LRO", floor, geometry_msgs::Polygon(), 0.0, 0.0);
	places::Desk desk(5, "Adriano's Desk", office, geometry_msgs::Polygon(), 0.0, 0.0);

	/*** Agent Related Classes ***/
	agents::Computer computer(7, "adrianohrl_pc");
	agents::Person person(8, "Adriano Henrique Rossette Leite");
	agents::Robot robot(9, "Pioneer P3-DX", true);
	agents::VoiceCommander voice_commander(10, "Adriano Henrique Rossette Leite", "adrianohrl", computer);

	ros::NodeHandle nh;
	::mrta_vc::SystemManagerNode node(nh);
	node.spin();
	return 0;
}
