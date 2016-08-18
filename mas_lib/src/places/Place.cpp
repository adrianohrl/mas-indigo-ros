/**
 *  Place.h
 *
 *  Version: 1.2.2
 *  Created on: 03/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/places/Place.h"

/**
 *
 */
unifei::expertinos::mrta_vc::places::Place::Place(int id, std::string name, geometry_msgs::Polygon boundary, double x, double y, double theta) : unifei::expertinos::mrta_vc::places::Location(x, y, theta)
{
	id_ = id;
	name_ = name;
	boundary_ = boundary;
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Place::Place(int id, std::string name, geometry_msgs::Polygon boundary, geometry_msgs::Pose pose_msg) : unifei::expertinos::mrta_vc::places::Location(pose_msg)
{
	id_ = id;
	name_ = name;
	boundary_ = boundary;
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Place::Place(const mas_msgs::Place::ConstPtr& place_msg) : unifei::expertinos::mrta_vc::places::Location(place_msg->location) 
{
	if (!isValidType(place_msg->type)) 
	{
		return;
	}
	id_ = place_msg->id;
	name_ = place_msg->name;
	boundary_ = place_msg->boundary;
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Place::Place(mas_msgs::Place place_msg) : unifei::expertinos::mrta_vc::places::Location(place_msg.location)
{
	if (!isValidType(place_msg.type)) 
	{
		return;
	}
	id_ = place_msg.id;
	name_ = place_msg.name;
	boundary_ = place_msg.boundary;	
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Place::~Place() 
{
}

/**
 *
 */
int unifei::expertinos::mrta_vc::places::Place::getId() 
{
	return id_;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::places::Place::getName() 
{
	return name_;
}

/**
 *
 */
geometry_msgs::Polygon unifei::expertinos::mrta_vc::places::Place::getBoundary() 
{
	return boundary_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::places::Place::isValidType(int type) 
{
	return type == getType();
}

/**
 *
 */
int unifei::expertinos::mrta_vc::places::Place::getType() 
{
	return PLACE;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::places::Place::setBoundary(geometry_msgs::Polygon boundary) 
{
	boundary_ = boundary;
}

/**
 *
 */
mas_msgs::Place unifei::expertinos::mrta_vc::places::Place::toMsg() 
{
	mas_msgs::Place place_msg;
	place_msg.type = getType();
	place_msg.id = id_;
	place_msg.name = name_;
	place_msg.location = Location::toMsg();
	place_msg.boundary = boundary_;
	return place_msg;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::places::Place::operator==(const unifei::expertinos::mrta_vc::places::Place& place)
{
	return id_ == place.id_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::places::Place::operator!=(const unifei::expertinos::mrta_vc::places::Place& place) 
{
	return id_ != place.id_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::places::Place::operator=(const unifei::expertinos::mrta_vc::places::Place& place) 
{
	unifei::expertinos::mrta_vc::places::Location::operator=(place);
	id_ = place.id_;
	name_ = place.name_;
	boundary_ = place.boundary_;
}
