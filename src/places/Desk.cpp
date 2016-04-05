/**
 *  Desk.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/places/Desk.h"

/**
 *
 */
unifei::expertinos::mrta_vc::places::Desk::Desk(int id, std::string name, Office office, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta), office_(office)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Desk::Desk(int id, std::string name, Office office, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg), office_(office)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Desk::Desk(const ::mrta_vc::Place::ConstPtr& desk_msg) : Place(desk_msg), office_(::mrta_vc::Place()) //desk_(desk_msg->parent) 
{
	// verificar se tem o office 
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Desk::Desk(::mrta_vc::Place desk_msg) : Place(desk_msg), office_(::mrta_vc::Place()) //desk_(desk_msg.parent) 
{
	// verificar se tem o office
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Desk::~Desk() 
{
}

/**
 *
 */
int unifei::expertinos::mrta_vc::places::Desk::getType() 
{
	return DESK;
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Office unifei::expertinos::mrta_vc::places::Desk::getOffice() 
{
	return office_;
}


/**
 *
 */
::mrta_vc::Place unifei::expertinos::mrta_vc::places::Desk::toMsg() 
{
	::mrta_vc::Place office_msg = Place::toMsg();
	//office_msg.office = office_.toMsg();
	return office_msg;
}
