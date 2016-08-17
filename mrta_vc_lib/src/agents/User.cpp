/**
 *  VoiceCommander.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/agents/User.h"

/**
 *
 */
unifei::expertinos::mrta_vc::agents::User::User() : unifei::expertinos::mrta_vc::agents::Person(), computer_(::mrta_vc::Agent())
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::User::User(int id, std::string name, unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level, std::string login_name, unifei::expertinos::mrta_vc::agents::Computer computer, double x, double y, double theta) : unifei::expertinos::mrta_vc::agents::Person(id, name, hierarchy_level, x, y, theta), computer_(computer)
{
	login_name_ = login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::User::User(int id, std::string name, unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level, std::string login_name, unifei::expertinos::mrta_vc::agents::Computer computer, geometry_msgs::Pose pose_msg) : unifei::expertinos::mrta_vc::agents::Person(id, name, hierarchy_level, pose_msg), computer_(computer)
{
	login_name_ = login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::User::User(int id, std::string name, unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level, std::string login_name, unifei::expertinos::mrta_vc::agents::Computer computer, unifei::expertinos::mrta_vc::places::Location location) : unifei::expertinos::mrta_vc::agents::Person(id, name, hierarchy_level, location), computer_(computer)
{
	login_name_ = login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::User::User(const ::mrta_vc::Agent::ConstPtr& user_msg) : unifei::expertinos::mrta_vc::agents::Person(user_msg), computer_(::mrta_vc::Agent())
{
  login_name_ = user_msg->login_name;
	::mrta_vc::Agent computer_msg;
	computer_msg.type = COMPUTER;
	computer_msg.id = user_msg->computer_id;
	computer_msg.name = user_msg->computer_hostname;
	computer_msg.mobile = user_msg->computer_mobile;
	computer_msg.location = user_msg->computer_location;
	computer_ = unifei::expertinos::mrta_vc::agents::Computer(computer_msg);
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::User::User(::mrta_vc::Agent user_msg) : unifei::expertinos::mrta_vc::agents::Person(user_msg), computer_(::mrta_vc::Agent())
{
  login_name_ = user_msg.login_name;
	::mrta_vc::Agent computer_msg;
	computer_msg.type = COMPUTER;
	computer_msg.id = user_msg.computer_id;
	computer_msg.name = user_msg.computer_hostname;
	computer_msg.mobile = user_msg.computer_mobile;
	computer_msg.location = user_msg.computer_location;
	computer_ = unifei::expertinos::mrta_vc::agents::Computer(computer_msg);
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::User::~User()
{
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::User::getLoginName() 
{
	return login_name_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Computer unifei::expertinos::mrta_vc::agents::User::getComputer() 
{
	return computer_;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::agents::User::getLastBeaconTimestamp() 
{
	return last_beacon_timestamp_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::User::isValid(std::string password) 
{
	return password_ == password;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::User::isLogged() 
{
	return (ros::Time::now() - last_beacon_timestamp_).toSec() <= MAXIMUM_USER_BEACON_ABSENCE_DURATION;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::User::isNotLoggedAnyMore(User user)
{
	return !user.isLogged();
}


/**
 *
 */
int unifei::expertinos::mrta_vc::agents::User::getType() 
{
	return USER;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::User::getClassName() 
{
  return "user";
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::User::setLoginName(std::string login_name) 
{
	login_name_ = login_name;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::User::setPassword(std::string password) 
{
	password_ = password;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::User::setComputer(unifei::expertinos::mrta_vc::agents::Computer computer) 
{
	computer_ = computer;
	unifei::expertinos::mrta_vc::agents::Agent::setLocation(computer.getLocation());
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::User::setLastBeaconTimestamp(ros::Time last_beacon_timestamp) 
{
	if (last_beacon_timestamp > last_beacon_timestamp_ && last_beacon_timestamp <= ros::Time::now()) 
	{
		last_beacon_timestamp_ = last_beacon_timestamp;
	}
}

/**
 *
 */
::mrta_vc::Agent unifei::expertinos::mrta_vc::agents::User::toMsg()
{
	::mrta_vc::Agent user_msg = unifei::expertinos::mrta_vc::agents::Person::toMsg();
	user_msg.login_name = login_name_;
	::mrta_vc::Agent computer_msg = computer_.toMsg();
	user_msg.computer_id = computer_msg.id;
	user_msg.computer_hostname = computer_msg.name;
	user_msg.computer_mobile = computer_msg.mobile;
	user_msg.computer_location = computer_msg.location;
	return user_msg;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::User::toString() 
{
  std::string aux = unifei::expertinos::mrta_vc::agents::Person::toString();
	return aux.substr(0, aux.length() - 1) +
			", login: " + login_name_ +
			", " + computer_.toString() +
			"}";
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::User::operator=(const unifei::expertinos::mrta_vc::agents::User& user)
{
	unifei::expertinos::mrta_vc::agents::Person::operator=(user);
	login_name_ = user.login_name_;
	computer_ = user.computer_;
}
