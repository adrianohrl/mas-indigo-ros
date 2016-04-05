/**
 *  Building.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/places/Building.h"

/**
 *
 */
unifei::expertinos::mrta_vc::places::Building::Building(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta), campus_(campus)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Building::Building(int id, std::string name, Campus campus, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg), campus_(campus)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Building::Building(const ::mrta_vc::Place::ConstPtr& building_msg) : Place(building_msg), campus_(::mrta_vc::Place()) //campus_(building_msg->parent) 
{
	// verificar se tem o campus 
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Building::Building(::mrta_vc::Place building_msg) : Place(building_msg), campus_(::mrta_vc::Place()) //campus_(building_msg.parent) 
{
	// verificar se tem o campus
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Building::~Building() 
{
}

/**
 *
 */
int unifei::expertinos::mrta_vc::places::Building::getType() 
{
	return BUILDING;
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus unifei::expertinos::mrta_vc::places::Building::getCampus() 
{
	return campus_;
}


/**
 *
 */
::mrta_vc::Place unifei::expertinos::mrta_vc::places::Building::toMsg() 
{
	::mrta_vc::Place building_msg = Place::toMsg();
	//building_msg.campus = campus_.toMsg();
	return building_msg;
}
