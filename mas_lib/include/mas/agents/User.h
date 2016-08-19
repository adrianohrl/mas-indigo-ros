/**
 *  User.h
 *
 *  Version: 1.2.4
 *  Created on: 05/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef AGENTS_USER_H_
#define AGENTS_USER_H_

#include <ros/ros.h>
#include <mas_msgs/User.h>
#include "mas/agents/Person.h"
#include "mas/agents/Computer.h"

#define USER_BEACON_INTERVAL_DURATION 5.0
#define MAXIMUM_USER_BEACON_ABSENCE_DURATION 3 * USER_BEACON_INTERVAL_DURATION

namespace mas 
{
	namespace agents
	{
		class User : public Person
		{

		public:
			User();
			User(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, double x = 0.0, double y = 0.0, double theta = 0.0);
			User(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, geometry_msgs::Pose pose_msg);
			User(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, mas::places::Location location);
			User(const mas_msgs::Agent::ConstPtr& user_msg);
			User(mas_msgs::Agent user_msg);
			virtual ~User();

			std::string getLoginName();
			Computer getComputer();
			ros::Time getLastBeaconTimestamp();
			bool isValid(std::string password);
			bool isLogged();
			static bool isNotLoggedAnyMore(User user);
			void setComputer(Computer computer);
			void setLastBeaconTimestamp(ros::Time last_beacon_timestamp = ros::Time::now());
			virtual mas_msgs::Agent toMsg();
			virtual std::string toString();
			virtual void operator=(const User& user);

		protected:
			virtual int getType();
			void setLoginName(std::string login_name);
			void setPassword(std::string password);

		private:
			std::string login_name_;
			std::string password_;
			Computer computer_;
			ros::Time last_beacon_timestamp_;
			
			virtual std::string getClassName();

		};
	}
}		

#endif /* AGENTS_USER_H_ */
