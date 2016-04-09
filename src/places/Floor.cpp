/**
 *  Floor.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/places/Floor.h"

/**
 *
 */
unifei::expertinos::mrta_vc::places::Floor::Floor(int id, std::string name, Building building, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta), building_(building)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Floor::Floor(int id, std::string name, Building building, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg), building_(building)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Floor::Floor(const ::mrta_vc::Place::ConstPtr& floor_msg) : Place(floor_msg), building_(::mrta_vc::Place()) //floor_(floor_msg->parent) 
{
	// verificar se tem o building 
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Floor::Floor(::mrta_vc::Place floor_msg) : Place(floor_msg), building_(::mrta_vc::Place()) //floor_(floor_msg.parent) 
{
	// verificar se tem o building
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Floor::~Floor() 
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Building unifei::expertinos::mrta_vc::places::Floor::getBuilding() 
{
	return building_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::places::Floor::getType() 
{
	return FLOOR;
}


/**
 *
 */
::mrta_vc::Place unifei::expertinos::mrta_vc::places::Floor::toMsg() 
{
	::mrta_vc::Place building_msg = Place::toMsg();
	//building_msg.building = building_.toMsg();
	return building_msg;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::places::Floor::operator=(const unifei::expertinos::mrta_vc::places::Floor& floor)
{
	unifei::expertinos::mrta_vc::places::Place::operator=(floor);
	building_ = floor.building_;
}
