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
          Computer();
					Computer(int id, std::string hostname, bool mobile = false, double x = 0.0, double y = 0.0, double theta = 0.0);
					Computer(int id, std::string hostname, bool mobile, geometry_msgs::Pose pose_msg);
					Computer(int id, std::string hostname, bool mobile, unifei::expertinos::mrta_vc::places::Location location);
					Computer(const ::mrta_vc::Agent::ConstPtr& computer_msg);
					Computer(::mrta_vc::Agent computer_msg);		
					virtual ~Computer();

					std::string getHostname();
					bool isMobile();
          virtual ::mrta_vc::Agent toMsg();
          virtual std::string toString();
          virtual bool equals(Computer computer);
          virtual bool operator==(const Computer& computer);
          virtual void operator=(const Computer& computer);
					
        protected:
          virtual int getType();
					void setHostname(std::string hostname);
					void setMobile(bool mobile);

				private:
					std::string hostname_;
					bool mobile_;
					
          virtual std::string getClassName();

				};
			}
		}
	}
}		
					
#endif /* COMPUTER_H_ */
