/**
 *  Agent.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/agents/Agent.h"

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Agent::Agent(int id, double x, double y, double theta) : location_(x, y, theta)
{
	id_ = id;	
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Agent::Agent(int id, geometry_msgs::Pose pose_msg) : location_(pose_msg)
{
	id_ = id;	
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Agent::Agent(const ::mrta_vc::Agent::ConstPtr& agent_msg) : location_(agent_msg->location)
{
	id_ = agent_msg->id;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Agent::Agent(::mrta_vc::Agent agent_msg) : location_(agent_msg.location)
{
	id_ = agent_msg.id;		
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Agent::~Agent() 
{
}

/**
 *
 */
int unifei::expertinos::mrta_vc::agents::Agent::getId() 
{
	return id_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::places::Location unifei::expertinos::mrta_vc::agents::Agent::getLocation() 
{
	return location_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Agent::isValidType(int type) 
{
	return type == getType();
}

/**
 *
 */
int unifei::expertinos::mrta_vc::agents::Agent::getType() 
{
	return AGENT;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Agent::setLocation(double x, double y, double theta) 
{
	location_.setPose(x, y, theta);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Agent::setLocation(geometry_msgs::Pose pose_msg) 
{
	location_.setPose(pose_msg);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Agent::setLocation(unifei::expertinos::mrta_vc::places::Location location) 
{
	location_.setPose(location);
}

/**
 *
 */
::mrta_vc::Agent unifei::expertinos::mrta_vc::agents::Agent::toMsg() 
{
	::mrta_vc::Agent agent_msg;
	agent_msg.id = id_;
	agent_msg.location = location_.toMsg();
	return agent_msg;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Agent::equals(Agent agent) 
{
	return id_ == agent.id_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Agent::operator==(const unifei::expertinos::mrta_vc::agents::Agent& agent)
{
	return id_ == agent.id_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Agent::operator!=(const unifei::expertinos::mrta_vc::agents::Agent& agent) 
{
	return id_ != agent.id_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Agent::operator=(const unifei::expertinos::mrta_vc::agents::Agent& agent) 
{
	id_ = agent.id_;
	location_ = agent.location_;
}
