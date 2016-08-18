/**
 *  Floor.h
 *
 *  Version: 1.2.2
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/places/Floor.h"

/**
 *
 */
unifei::expertinos::mrta_vc::places::Floor::Floor(int id, std::string name, unifei::expertinos::mrta_vc::places::Building building, geometry_msgs::Polygon boundary, double x, double y, double theta) : unifei::expertinos::mrta_vc::places::Place(id, name, boundary, x, y, theta), building_(building)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Floor::Floor(int id, std::string name, unifei::expertinos::mrta_vc::places::Building building, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : unifei::expertinos::mrta_vc::places::Place(id, name, boundary, pose_msg), building_(building)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Floor::Floor(const mas_msgs::Place::ConstPtr& floor_msg) : unifei::expertinos::mrta_vc::places::Place(floor_msg), building_(mas_msgs::Place()) //floor_(floor_msg->parent) 
{
	// verificar se tem o building 
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Floor::Floor(mas_msgs::Place floor_msg) : unifei::expertinos::mrta_vc::places::Place(floor_msg), building_(mas_msgs::Place()) //floor_(floor_msg.parent) 
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
mas_msgs::Place unifei::expertinos::mrta_vc::places::Floor::toMsg() 
{
	mas_msgs::Place building_msg = Place::toMsg();
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
