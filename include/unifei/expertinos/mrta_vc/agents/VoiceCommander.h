/**
 *  VoiceCommander.h
 *
 *  Version: 1.0.0.0
 *  Created on: 05/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef VOICE_COMMANDER_H_
#define VOICE_COMMANDER_H_

#include "unifei/expertinos/mrta_vc/agents/Person.h"
#include "unifei/expertinos/mrta_vc/agents/Computer.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace agents
			{
				class VoiceCommander : public Person
				{

				public:
					VoiceCommander(int id, std::string name, std::string login_name, Computer computer, double x = 0, double y = 0, double theta = 0);
					VoiceCommander(int id, std::string name, std::string login_name, Computer computer, geometry_msgs::Pose pose_msg);
					VoiceCommander(const ::mrta_vc::Agent::ConstPtr& voice_commander_msg);
					VoiceCommander(::mrta_vc::Agent voice_commander_msg);		
					~VoiceCommander();

					std::string getLoginName();
					Computer getComputer();
					void setLoginName(std::string login_name);
					void setComputer(Computer computer);
					::mrta_vc::Agent toMsg();
					void operator=(const VoiceCommander& VoiceCommander);
					
				protected:
					int getType();

				private:
					std::string login_name_;
					Computer computer_;

				};
			}
		}
	}
}		
					
#endif /* VOICE_COMMANDER_H_ */
