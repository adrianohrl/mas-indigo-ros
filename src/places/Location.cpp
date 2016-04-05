/**
 *  Location.h
 *
 *  Version: 1.0.0.0
 *  Created on: 03/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/places/Location.h"

/**
 *
 */
unifei::expertinos::mrta_vc::places::Location::Location(double x, double y, double theta)
{
	x_ = x;
	y_ = y;
	theta_ = theta;
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Location::Location(geometry_msgs::Pose pose_msg)
{
	x_ = pose_msg.position.x;
	y_ = pose_msg.position.y;
	theta_ = tf::getYaw(pose_msg.orientation);
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Location::Location(const ::mrta_vc::Location::ConstPtr& location_msg) 
{
	x_ = location_msg->x;
	y_ = location_msg->y;
	theta_ = location_msg->theta;	
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Location::Location(::mrta_vc::Location location_msg) 
{
	x_ = location_msg.x;
	y_ = location_msg.y;
	theta_ = location_msg.theta;	
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Location::~Location() 
{
}

/**
 *
 */
double unifei::expertinos::mrta_vc::places::Location::getX() 
{
	return x_;
}

/**
 *
 */
double unifei::expertinos::mrta_vc::places::Location::getY() 
{
	return y_;
}

/**
 *
 */
double unifei::expertinos::mrta_vc::places::Location::getTheta() 
{
	return theta_;
}

/**
 *
 */
geometry_msgs::Pose unifei::expertinos::mrta_vc::places::Location::getPose() 
{
	geometry_msgs::Pose pose_msg;
	pose_msg.position.x = x_;
	pose_msg.position.y = y_;
	pose_msg.orientation = tf::createQuaternionMsgFromYaw(theta_);
	return pose_msg;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::places::Location::setPose(double x, double y, double theta) 
{
	x_ = x;
	y_ = y;
	theta_ = theta;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::places::Location::setPose(geometry_msgs::Pose pose_msg) 
{
	x_ = pose_msg.position.x;
	y_ = pose_msg.position.y;
	theta_ = tf::getYaw(pose_msg.orientation);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::places::Location::setPose(::mrta_vc::Location location_msg) 
{
	x_ = location_msg.x;
	y_ = location_msg.y;
	theta_ = location_msg.theta;
}

/**
 *
 */
::mrta_vc::Location unifei::expertinos::mrta_vc::places::Location::toMsg() 
{
	::mrta_vc::Location location_msg;
	location_msg.x = x_;
	location_msg.y = y_;
	location_msg.theta = theta_;
	return location_msg;
}
