/**
 *  ExternalArea.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/places/ExternalArea.h"

/**
 *
 */
unifei::expertinos::mrta_vc::places::ExternalArea::ExternalArea(int id, std::string name, unifei::expertinos::mrta_vc::places::Campus campus, geometry_msgs::Polygon boundary, double x, double y, double theta) : unifei::expertinos::mrta_vc::places::Place(id, name, boundary, x, y, theta), campus_(campus)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::ExternalArea::ExternalArea(int id, std::string name, unifei::expertinos::mrta_vc::places::Campus campus, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : unifei::expertinos::mrta_vc::places::Place(id, name, boundary, pose_msg), campus_(campus)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::ExternalArea::ExternalArea(const ::mrta_vc::Place::ConstPtr& external_area_msg) : unifei::expertinos::mrta_vc::places::Place(external_area_msg), campus_(::mrta_vc::Place())//campus_(external_area_msg->parent)
{
	// verificar se tem o campus 
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::ExternalArea::ExternalArea(::mrta_vc::Place external_area_msg) : unifei::expertinos::mrta_vc::places::Place(external_area_msg), campus_(::mrta_vc::Place())//campus_(external_area_msg.parent)
{
	// verificar se tem o campus
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::ExternalArea::~ExternalArea() 
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus unifei::expertinos::mrta_vc::places::ExternalArea::getCampus() 
{
	return campus_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::places::ExternalArea::getType() 
{
	return EXTERNAL_AREA;
}


/**
 *
 */
::mrta_vc::Place unifei::expertinos::mrta_vc::places::ExternalArea::toMsg() 
{
	::mrta_vc::Place external_area_msg = Place::toMsg();
	// external_area_msg.campus = campus_.toMsg();
	return external_area_msg;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::places::ExternalArea::operator=(const unifei::expertinos::mrta_vc::places::ExternalArea& external_area)
{
	unifei::expertinos::mrta_vc::places::Place::operator=(external_area);
	campus_ = external_area.campus_;
}
