/**
 *  Agent.cpp
 *
 *  Version: 1.2.2
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/agents/Agent.h"

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Agent::Agent()
{	
}

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
unifei::expertinos::mrta_vc::agents::Agent::Agent(int id, unifei::expertinos::mrta_vc::places::Location location) : location_(location)
{
	id_ = id;	
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Agent::Agent(const mas_msgs::Agent::ConstPtr& agent_msg) : location_(agent_msg->location)
{
	id_ = agent_msg->id;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Agent::Agent(mas_msgs::Agent agent_msg) : location_(agent_msg.location)
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
	return AGENT_;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::Agent::getClassName() 
{
  return "agent";
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Agent::setId(int id) 
{
	id_ = id;
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
mas_msgs::Agent unifei::expertinos::mrta_vc::agents::Agent::toMsg() 
{
	mas_msgs::Agent agent_msg;
	agent_msg.id = id_;
	agent_msg.location = location_.toMsg();
	return agent_msg;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::Agent::toString() 
{
	std::stringstream aux;
  aux << getClassName() << ": {id: " << id_;
	return aux.str();
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Agent::equals(Agent agent) 
{
	return operator==(agent);
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
	return !operator==(agent);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Agent::operator=(const unifei::expertinos::mrta_vc::agents::Agent& agent) 
{
	id_ = agent.id_;
	location_ = agent.location_;
}
