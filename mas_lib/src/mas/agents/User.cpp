/**
 *  VoiceCommander.cpp
 *
 *  Version: 1.2.4
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/User.h"

namespace mas
{
	namespace agents
	{

		/**
		 *
		 */
		User::User() : Person(), computer_(mas_msgs::Agent())
		{
		}

		/**
		 *
		 */
		User::User(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, double x, double y, double theta) : Person(id, name, hierarchy_level, x, y, theta), computer_(computer)
		{
			login_name_ = login_name;
		}

		/**
		 *
		 */
		User::User(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, geometry_msgs::Pose pose_msg) : Person(id, name, hierarchy_level, pose_msg), computer_(computer)
		{
			login_name_ = login_name;
		}

		/**
		 *
		 */
		User::User(int id, std::string name, HierarchyLevelEnum hierarchy_level, std::string login_name, Computer computer, places::Location location) : Person(id, name, hierarchy_level, location), computer_(computer)
		{
			login_name_ = login_name;
		}

		/**
		 *
		 */
		User::User(const mas_msgs::Agent::ConstPtr& user_msg) : Person(user_msg), computer_(mas_msgs::Agent())
		{
		  login_name_ = user_msg->login_name;
			mas_msgs::Agent computer_msg;
			computer_msg.type = COMPUTER;
			computer_msg.id = user_msg->computer_id;
			computer_msg.name = user_msg->computer_hostname;
			computer_msg.mobile = user_msg->computer_mobile;
			computer_msg.location = user_msg->computer_location;
			computer_ = Computer(computer_msg);
		}

		/**
		 *
		 */
		User::User(mas_msgs::Agent user_msg) : Person(user_msg), computer_(mas_msgs::Agent())
		{
		  login_name_ = user_msg.login_name;
			mas_msgs::Agent computer_msg;
			computer_msg.type = COMPUTER;
			computer_msg.id = user_msg.computer_id;
			computer_msg.name = user_msg.computer_hostname;
			computer_msg.mobile = user_msg.computer_mobile;
			computer_msg.location = user_msg.computer_location;
			computer_ = Computer(computer_msg);
		}

		/**
		 *
		 */
		User::~User()
		{
		}

		/**
		 *
		 */
		std::string User::getLoginName() 
		{
			return login_name_;
		}

		/**
		 *
		 */
		Computer User::getComputer() 
		{
			return computer_;
		}

		/**
		 *
		 */
		ros::Time User::getLastBeaconTimestamp() 
		{
			return last_beacon_timestamp_;
		}

		/**
		 *
		 */
		bool User::isValid(std::string password) 
		{
			return password_ == password;
		}

		/**
		 *
		 */
		bool User::isLogged() 
		{
			return (ros::Time::now() - last_beacon_timestamp_).toSec() <= MAXIMUM_USER_BEACON_ABSENCE_DURATION;
		}

		/**
		 *
		 */
		bool User::isNotLoggedAnyMore(User user)
		{
			return !user.isLogged();
		}


		/**
		 *
		 */
		int User::getType() 
		{
			return USER;
		}

		/**
		 *
		 */
		std::string User::getClassName() 
		{
		  return "user";
		}

		/**
		 *
		 */
		void User::setLoginName(std::string login_name) 
		{
			login_name_ = login_name;
		}

		/**
		 *
		 */
		void User::setPassword(std::string password) 
		{
			password_ = password;
		}

		/**
		 *
		 */
		void User::setComputer(Computer computer) 
		{
			computer_ = computer;
			Agent::setLocation(computer.getLocation());
		}

		/**
		 *
		 */
		void User::setLastBeaconTimestamp(ros::Time last_beacon_timestamp) 
		{
			if (last_beacon_timestamp > last_beacon_timestamp_ && last_beacon_timestamp <= ros::Time::now()) 
			{
				last_beacon_timestamp_ = last_beacon_timestamp;
			}
		}

		/**
		 *
		 */
		mas_msgs::Agent User::toMsg()
		{
			mas_msgs::Agent user_msg = Person::toMsg();
			user_msg.login_name = login_name_;
			mas_msgs::Agent computer_msg = computer_.toMsg();
			user_msg.computer_id = computer_msg.id;
			user_msg.computer_hostname = computer_msg.name;
			user_msg.computer_mobile = computer_msg.mobile;
			user_msg.computer_location = computer_msg.location;
			return user_msg;
		}

		/**
		 *
		 */
		std::string User::toString() 
		{
		  std::string aux = Person::toString();
			return aux.substr(0, aux.length() - 1) +
					", login: " + login_name_ +
					", " + computer_.toString() +
					"}";
		}

		/**
		 *
		 */
		void User::operator=(const User& user)
		{
			Person::operator=(user);
			login_name_ = user.login_name_;
			computer_ = user.computer_;
		}
	
	}
}
