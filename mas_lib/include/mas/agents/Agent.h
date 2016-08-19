/**
 *  Agent.h
 *
 *  Version: 1.2.4
 *  Created on: 05/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef AGENTS_AGENT_H_
#define AGENTS_AGENT_H_

#include <sstream>
#include <string>
#include <mas_msgs/Agent.h>
#include "mas/places/Location.h"

#define NON_AGENT -1
#define AGENT_ 0
#define PERSON 1
#define COMPUTER 2
#define ROBOT 3
#define USER 4

namespace mas 
{
	namespace agents
	{
		class Agent 
		{

		public:
			Agent(int id, double x = 0.0, double y = 0.0, double theta = 0.0);
			Agent(int id, geometry_msgs::Pose pose_msg);
			Agent(int id, mas::places::Location location);
			Agent(const mas_msgs::Agent::ConstPtr& agent_msg);
			Agent(mas_msgs::Agent agent_msg);		
			virtual ~Agent();

			int getId();
			mas::places::Location getLocation();
			void setLocation(double x = 0.0, double y = 0.0, double theta = 0.0);
			void setLocation(geometry_msgs::Pose pose_msg);
			void setLocation(mas::places::Location location);
			virtual mas_msgs::Agent toMsg();
			virtual std::string toString();
			bool equals(Agent agent);
			virtual bool operator==(const Agent& agent);
			bool operator!=(const Agent& agent);
			virtual void operator=(const Agent& agent);

		protected:
			Agent();
		
			virtual int getType();
			void setId(int id);

		private:
			int id_;
			mas::places::Location location_;
			
			virtual bool isValidType(int type);
			virtual std::string getClassName();

		};
	}
}		
					
#endif /* AGENTS_AGENT_H_ */
