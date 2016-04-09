/**
 *  Computer.h
 *
 *  Version: 1.0.0.0
 *  Created on: 05/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef COMPUTER_H_
#define COMPUTER_H_

#include <string>
#include "unifei/expertinos/mrta_vc/agents/Agent.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace agents
			{
				class Computer : public Agent
				{

				public:
					Computer(int id, std::string hostname, bool mobile = false, double x = 0, double y = 0, double theta = 0);
					Computer(int id, std::string hostname, bool mobile, geometry_msgs::Pose pose_msg);
					Computer(const ::mrta_vc::Agent::ConstPtr& computer_msg);
					Computer(::mrta_vc::Agent computer_msg);		
					~Computer();

					std::string getHostname();
					bool isMobile();
					::mrta_vc::Agent toMsg();
					bool equals(Computer computer);
					bool operator==(const Computer& computer);
					bool operator!=(const Computer& computer);
					void operator=(const Computer& computer);
					
				protected:
					int getType();

				private:
					std::string hostname_;
					bool mobile_;

				};
			}
		}
	}
}		
					
#endif /* COMPUTER_H_ */
