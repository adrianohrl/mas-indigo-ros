/**
 *  VoiceCommander.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/agents/VoiceCommander.h"

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander() : unifei::expertinos::mrta_vc::agents::Person(), computer_(::mrta_vc::Agent())
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander(int id, std::string name, unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level, std::string login_name, unifei::expertinos::mrta_vc::agents::Computer computer, double x, double y, double theta) : unifei::expertinos::mrta_vc::agents::Person(id, name, hierarchy_level, x, y, theta), computer_(computer)
{
	login_name_ = login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander(int id, std::string name, unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level, std::string login_name, unifei::expertinos::mrta_vc::agents::Computer computer, geometry_msgs::Pose pose_msg) : unifei::expertinos::mrta_vc::agents::Person(id, name, hierarchy_level, pose_msg), computer_(computer)
{
	login_name_ = login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander(int id, std::string name, unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level, std::string login_name, unifei::expertinos::mrta_vc::agents::Computer computer, unifei::expertinos::mrta_vc::places::Location location) : unifei::expertinos::mrta_vc::agents::Person(id, name, hierarchy_level, location), computer_(computer)
{
	login_name_ = login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander(int id, std::string name, unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level, std::string login_name, unifei::expertinos::mrta_vc::agents::Computer computer) : unifei::expertinos::mrta_vc::agents::Person(id, name, hierarchy_level, computer.getLocation()), computer_(computer)
{
	login_name_ = login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander(const ::mrta_vc::Agent::ConstPtr& voice_commander_msg) : unifei::expertinos::mrta_vc::agents::Person(voice_commander_msg), computer_(::mrta_vc::Agent())
{
	login_name_ = voice_commander_msg->login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander(::mrta_vc::Agent voice_commander_msg) : unifei::expertinos::mrta_vc::agents::Person(voice_commander_msg), computer_(::mrta_vc::Agent())
{
	login_name_ = voice_commander_msg.login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::~VoiceCommander() 
{
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::VoiceCommander::getLoginName() 
{
	return login_name_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Computer unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer() 
{
	return computer_;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::agents::VoiceCommander::getLastBeaconTimestamp() 
{
	return last_beacon_timestamp_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::agents::VoiceCommander::getType() 
{
	return VOICE_COMMANDER;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::VoiceCommander::getClassName() 
{
	return "VOICE COMMANDER";
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::VoiceCommander::setLoginName(std::string login_name) 
{
	login_name_ = login_name;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::VoiceCommander::setPassword(std::string password) 
{
	password_ = password;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::VoiceCommander::setComputer(unifei::expertinos::mrta_vc::agents::Computer computer) 
{
	computer_ = computer;
	unifei::expertinos::mrta_vc::agents::Agent::setLocation(computer.getLocation());
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::VoiceCommander::setLastBeaconTimestamp(ros::Time last_beacon_timestamp) 
{
	if (last_beacon_timestamp > last_beacon_timestamp_ && last_beacon_timestamp <= ros::Time::now()) 
	{
		last_beacon_timestamp_ = last_beacon_timestamp;
	}
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::VoiceCommander::isValid(std::string password) 
{
	return password_ == password;
}

/**
 *
 */
::mrta_vc::Agent unifei::expertinos::mrta_vc::agents::VoiceCommander::toMsg() 
{
	::mrta_vc::Agent voice_commander_msg = unifei::expertinos::mrta_vc::agents::Person::toMsg();
	voice_commander_msg.login_name = login_name_;
	//voice_commander_msg.computer = computer_.toMsg();
	return voice_commander_msg;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::VoiceCommander::toString() 
{
	std::stringstream aux;
	aux << unifei::expertinos::mrta_vc::agents::Person::toString() << " - login: " << login_name_ << " - computer: " << computer_.getHostname();
	return aux.str();
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::VoiceCommander::operator=(const unifei::expertinos::mrta_vc::agents::VoiceCommander& voice_commander) 
{
	unifei::expertinos::mrta_vc::agents::Person::operator=(voice_commander);
	login_name_ = voice_commander.login_name_;
	//computer_ = voice_commander.computer_;
}
