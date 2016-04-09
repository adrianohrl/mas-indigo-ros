/**
 *  Robot.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/agents/Robot.h"

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile, double x, double y, double theta) : Computer(id, hostname, mobile, x, y, theta)
{
	holonomic_ = holonomic;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::Robot(int id, std::string hostname, bool holonomic, bool mobile, geometry_msgs::Pose pose_msg) : Computer(id, hostname, mobile, pose_msg)
{
	holonomic_ = holonomic;	
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::Robot(const ::mrta_vc::Agent::ConstPtr& robot_msg) : Computer(robot_msg)
{
	holonomic_ = robot_msg->holonomic;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::Robot(::mrta_vc::Agent robot_msg) : Computer(robot_msg)
{
	holonomic_ = robot_msg.holonomic;		
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot::~Robot() 
{
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Robot::isHolonomic() 
{
	return holonomic_;
}

/**
 *
 */
double unifei::expertinos::mrta_vc::agents::Robot::getVelX() 
{
	return vel_x_;
}

/**
 *
 */
double unifei::expertinos::mrta_vc::agents::Robot::getVelY() 
{
	return vel_y_;
}

/**
 *
 */
double unifei::expertinos::mrta_vc::agents::Robot::getVelTheta() 
{
	return vel_theta_;
}

/**
 *
 */
geometry_msgs::Twist unifei::expertinos::mrta_vc::agents::Robot::getVelocity() 
{
	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x = vel_x_;
	twist_msg.linear.y = vel_y_;
	twist_msg.angular.z = vel_theta_;
	return twist_msg;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::agents::Robot::getType() 
{
	return ROBOT;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::setVelocity(double x, double y, double theta) 
{
	vel_x_ = x;
	vel_y_= y;
	vel_theta_ = theta;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::setVelocity(geometry_msgs::Twist twist_msg) 
{
	vel_x_ = twist_msg.linear.x;
	vel_y_= twist_msg.linear.y;
	vel_theta_ = twist_msg.angular.z;
}

/**
 *
 */
::mrta_vc::Agent unifei::expertinos::mrta_vc::agents::Robot::toMsg() 
{
	::mrta_vc::Agent robot_msg = Computer::toMsg();
	robot_msg.holonomic = holonomic_;
	robot_msg.velocity.linear.x = vel_x_;
	robot_msg.velocity.linear.y = vel_y_;
	robot_msg.velocity.angular.z = vel_theta_;
	return robot_msg;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Robot::operator=(const unifei::expertinos::mrta_vc::agents::Robot& robot) 
{
	unifei::expertinos::mrta_vc::agents::Computer::operator=(robot);
	holonomic_ = robot.holonomic_;
	vel_x_ = robot.vel_x_;
	vel_y_ = robot.vel_y_;
	vel_theta_ = robot.vel_theta_;
}
