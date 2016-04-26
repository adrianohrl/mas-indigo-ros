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

#include <ros/ros.h>
#include "unifei/expertinos/mrta_vc/agents/Person.h"
#include "unifei/expertinos/mrta_vc/agents/Computer.h"

#define USER_BEACON_INTERVAL_DURATION 2.0

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
					VoiceCommander(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, double x, double y = 0.0, double theta = 0.0);
					VoiceCommander(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, geometry_msgs::Pose pose_msg);
					VoiceCommander(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, unifei::expertinos::mrta_vc::places::Location location);
					VoiceCommander(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer);
					VoiceCommander(const ::mrta_vc::Agent::ConstPtr& voice_commander_msg);
					VoiceCommander(::mrta_vc::Agent voice_commander_msg);		
					~VoiceCommander();

					std::string getLoginName();
					Computer getComputer();
					ros::Time getLastBeaconTimestamp();
					void setComputer(Computer computer);
					void setLastBeaconTimestamp(ros::Time last_beacon_timestamp = ros::Time::now());
					bool isValid(std::string password);
					::mrta_vc::Agent toMsg();
					std::string toString();
					void operator=(const VoiceCommander& VoiceCommander);
					
				protected:
					VoiceCommander();
					
					int getType();
					void setLoginName(std::string login_name);
					void setPassword(std::string password);

				private:
					std::string login_name_;
					std::string password_;
					Computer computer_;
					ros::Time last_beacon_timestamp_;
					
					std::string getClassName();

				};
			}
		}
	}
}		
					
#endif /* VOICE_COMMANDER_H_ */
