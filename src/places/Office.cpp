/**
 *  Office.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/places/Office.h"

/**
 *
 */
unifei::expertinos::mrta_vc::places::Office::Office(int id, std::string name, Floor floor, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta), floor_(floor)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Office::Office(int id, std::string name, Floor floor, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg), floor_(floor)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Office::Office(const ::mrta_vc::Place::ConstPtr& office_msg) : Place(office_msg), floor_(::mrta_vc::Place()) //office_(office_msg->parent) 
{
	// verificar se tem o floor 
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Office::Office(::mrta_vc::Place office_msg) : Place(office_msg), floor_(::mrta_vc::Place()) //office_(office_msg.parent) 
{
	// verificar se tem o floor
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Office::~Office() 
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Floor unifei::expertinos::mrta_vc::places::Office::getFloor() 
{
	return floor_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::places::Office::getType() 
{
	return OFFICE;
}


/**
 *
 */
::mrta_vc::Place unifei::expertinos::mrta_vc::places::Office::toMsg() 
{
	::mrta_vc::Place floor_msg = Place::toMsg();
	//floor_msg.floor = floor_.toMsg();
	return floor_msg;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::places::Office::operator=(const unifei::expertinos::mrta_vc::places::Office& office)
{
	unifei::expertinos::mrta_vc::places::Place::operator=(office);
	floor_ = office.floor_;
}
