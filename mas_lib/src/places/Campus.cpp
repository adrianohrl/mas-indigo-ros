/**
 *  Campus.h
 *
 *  Version: 1.2.2
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/places/Campus.h"

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus::Campus(int id, std::string name, geometry_msgs::Polygon boundary, double x, double y, double theta) : unifei::expertinos::mrta_vc::places::Place(id, name, boundary, x, y, theta)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus::Campus(int id, std::string name, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : unifei::expertinos::mrta_vc::places::Place(id, name, boundary, pose_msg)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus::Campus(const mas_msgs::Place::ConstPtr& campus_msg) : unifei::expertinos::mrta_vc::places::Place(campus_msg)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus::Campus(mas_msgs::Place campus_msg) : unifei::expertinos::mrta_vc::places::Place(campus_msg)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus::~Campus() 
{
}

/**
 *
 */
int unifei::expertinos::mrta_vc::places::Campus::getType() 
{
	return CAMPUS;
}
