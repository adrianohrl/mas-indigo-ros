/**
 *  Campus.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/places/Campus.h"

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus::Campus(int id, std::string name, geometry_msgs::Polygon boundary, double x, double y, double theta) : Place(id, name, boundary, x, y, theta)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus::Campus(int id, std::string name, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : Place(id, name, boundary, pose_msg)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus::Campus(const ::mrta_vc::Place::ConstPtr& campus_msg) : Place(campus_msg)
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Campus::Campus(::mrta_vc::Place campus_msg) : Place(campus_msg)
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
