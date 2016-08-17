/**
 *  Agent.h
 *
 *  Version: 1.0.0.0
 *  Created on: 05/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef AGENT_H_
#define AGENT_H_

#include <sstream>
#include <string>
#include "mrta_vc/Agent.h"
#include "unifei/expertinos/mrta_vc/places/Location.h"

#define NON_AGENT -1
#define AGENT_ 0
#define PERSON 1
#define COMPUTER 2
#define ROBOT 3
#define USER 4

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace agents
			{
				class Agent 
				{

				public:
					Agent(int id, double x = 0.0, double y = 0.0, double theta = 0.0);
					Agent(int id, geometry_msgs::Pose pose_msg);
					Agent(int id, unifei::expertinos::mrta_vc::places::Location location);
					Agent(const ::mrta_vc::Agent::ConstPtr& agent_msg);
					Agent(::mrta_vc::Agent agent_msg);		
					virtual ~Agent();

					int getId();
					unifei::expertinos::mrta_vc::places::Location getLocation();
					void setLocation(double x = 0.0, double y = 0.0, double theta = 0.0);
					void setLocation(geometry_msgs::Pose pose_msg);
					void setLocation(unifei::expertinos::mrta_vc::places::Location location);
          virtual ::mrta_vc::Agent toMsg();
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
					unifei::expertinos::mrta_vc::places::Location location_;
					
          virtual bool isValidType(int type);
          virtual std::string getClassName();

				};
			}
		}
	}
}		
					
#endif /* AGENT_H_ */
