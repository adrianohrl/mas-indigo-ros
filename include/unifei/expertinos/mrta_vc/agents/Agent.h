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

#include <string>
#include "mrta_vc/Agent.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace tasks
			{
				class Agent 
				{

				public:
					Agent(int id, std::string name, std::string description);
					Agent(const ::mrta_vc::Agent::ConstPtr& resource_msg);
					Agent(::mrta_vc::Agent resource_msg);		
					~Agent();

					int getId();
					std::string getName();
					std::string getDescription();
					void setDescription(std::string description);
					bool equals(Agent resource);
					::mrta_vc::Agent toMsg();
					bool operator==(const Agent& resource);
					bool operator!=(const Agent& resource);

				private:
					int id_;
					std::string name_;
					std::string description_;	

				};
			}
		}
	}
}		
					
#endif /* AGENT_H_ */
