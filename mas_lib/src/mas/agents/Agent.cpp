/**
 *  Agent.cpp
 *
 *  Version: 1.2.4
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/Agent.h"

namespace mas
{
	namespace agents
	{

		/**
		 *
		 */
		Agent::Agent()
		{	
		}

		/**
		 *
		 */
		Agent::Agent(int id, double x, double y, double theta) : location_(x, y, theta)
		{
			id_ = id;	
		}

		/**
		 *
		 */
		Agent::Agent(int id, geometry_msgs::Pose pose_msg) : location_(pose_msg)
		{
			id_ = id;	
		}

		/**
		 *
		 */
		Agent::Agent(int id, places::Location location) : location_(location)
		{
			id_ = id;	
		}

		/**
		 *
		 */
		Agent::Agent(const mas_msgs::Agent::ConstPtr& agent_msg) : location_(agent_msg->location)
		{
			id_ = agent_msg->id;
		}

		/**
		 *
		 */
		Agent::Agent(mas_msgs::Agent agent_msg) : location_(agent_msg.location)
		{
			id_ = agent_msg.id;		
		}

		/**
		 *
		 */
		Agent::~Agent() 
		{
		}

		/**
		 *
		 */
		int Agent::getId() 
		{
			return id_;
		}

		/**
		 *
		 */
		places::Location Agent::getLocation() 
		{
			return location_;
		}

		/**
		 *
		 */
		bool Agent::isValidType(int type) 
		{
			return type == getType();
		}

		/**
		 *
		 */
		int Agent::getType() 
		{
			return AGENT_;
		}

		/**
		 *
		 */
		std::string Agent::getClassName() 
		{
		  return "agent";
		}

		/**
		 *
		 */
		void Agent::setId(int id) 
		{
			id_ = id;
		}

		/**
		 *
		 */
		void Agent::setLocation(double x, double y, double theta) 
		{
			location_.setPose(x, y, theta);
		}

		/**
		 *
		 */
		void Agent::setLocation(geometry_msgs::Pose pose_msg) 
		{
			location_.setPose(pose_msg);
		}

		/**
		 *
		 */
		void Agent::setLocation(places::Location location) 
		{
			location_.setPose(location);
		}

		/**
		 *
		 */
		mas_msgs::Agent Agent::toMsg() 
		{
			mas_msgs::Agent agent_msg;
			agent_msg.id = id_;
			agent_msg.location = location_.toMsg();
			return agent_msg;
		}

		/**
		 *
		 */
		std::string Agent::toString() 
		{
			std::stringstream aux;
		  aux << getClassName() << ": {id: " << id_;
			return aux.str();
		}

		/**
		 *
		 */
		bool Agent::equals(Agent agent) 
		{
			return operator==(agent);
		}

		/**
		 *
		 */
		bool Agent::operator==(const Agent& agent)
		{
			return id_ == agent.id_;
		}

		/**
		 *
		 */
		bool Agent::operator!=(const Agent& agent) 
		{
			return !operator==(agent);
		}

		/**
		 *
		 */
		void Agent::operator=(const Agent& agent) 
		{
			id_ = agent.id_;
			location_ = agent.location_;
		}
	
	}
}
