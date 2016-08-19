/**
 *  Computer.h
 *
 *  Version: 1.2.4
 *  Created on: 05/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef AGENTS_COMPUTER_H_
#define AGENTS_COMPUTER_H_

#include <mas_msgs/Computer.h>
#include "mas/agents/Agent.h"

namespace mas 
{
	namespace agents
	{
		class Computer : public Agent
		{

		public:
			Computer();
			Computer(int id, std::string hostname, bool mobile = false, double x = 0.0, double y = 0.0, double theta = 0.0);
			Computer(int id, std::string hostname, bool mobile, geometry_msgs::Pose pose_msg);
			Computer(int id, std::string hostname, bool mobile, mas::places::Location location);
			Computer(const mas_msgs::Agent::ConstPtr& computer_msg);
			Computer(mas_msgs::Agent computer_msg);
			virtual ~Computer();

			std::string getHostname();
			bool isMobile();
			virtual mas_msgs::Agent toMsg();
			virtual std::string toString();
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

#endif /* AGENTS_COMPUTER_H_ */
