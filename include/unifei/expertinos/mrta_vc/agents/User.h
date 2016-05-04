/**
 *  User.h
 *
 *  Version: 1.0.0.0
 *  Created on: 05/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef USER_H_
#define USER_H_

#include <ros/ros.h>
#include "unifei/expertinos/mrta_vc/agents/Person.h"
#include "unifei/expertinos/mrta_vc/agents/Computer.h"

#define USER_BEACON_INTERVAL_DURATION 5.0
#define MAXIMUM_USER_BEACON_ABSENCE_DURATION 3 * USER_BEACON_INTERVAL_DURATION

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace agents
			{
        class User : public Person
				{

				public:
          User(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, double x, double y = 0.0, double theta = 0.0);
          User(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, geometry_msgs::Pose pose_msg);
          User(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, unifei::expertinos::mrta_vc::places::Location location);
          User(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer);
          User(const ::mrta_vc::Agent::ConstPtr& user_msg);
          User(::mrta_vc::Agent user_msg);
          ~User();

          std::string getLoginName();
          Computer getComputer();
          ros::Time getLastBeaconTimestamp();
          bool isValid(std::string password);
          bool isLogged();
          static bool isNotLoggedAnyMore(User user);
          void setComputer(Computer computer);
          void setLastBeaconTimestamp(ros::Time last_beacon_timestamp = ros::Time::now());
          ::mrta_vc::Agent toMsg();
          std::string toString();
          void operator=(const User& user);
          		
        protected:
          User();

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
					
#endif /* USER_H_ */
