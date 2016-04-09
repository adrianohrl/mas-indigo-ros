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
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander(int id, std::string name, std::string login_name, Computer computer, double x, double y, double theta) : Person(id, name, x, y, theta), computer_(computer)
{
	login_name_ = login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander(int id, std::string name, std::string login_name, Computer computer, geometry_msgs::Pose pose_msg) : Person(id, name, pose_msg), computer_(computer)
{
	login_name_ = login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander(const ::mrta_vc::Agent::ConstPtr& voice_commander_msg) : Person(voice_commander_msg), computer_(::mrta_vc::Agent())
{
	login_name_ = voice_commander_msg->login_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander::VoiceCommander(::mrta_vc::Agent voice_commander_msg) : Person(voice_commander_msg), computer_(::mrta_vc::Agent())
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
int unifei::expertinos::mrta_vc::agents::VoiceCommander::getType() 
{
	return VOICE_COMMANDER;
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
void unifei::expertinos::mrta_vc::agents::VoiceCommander::setComputer(unifei::expertinos::mrta_vc::agents::Computer computer) 
{
	computer_ = computer;
}

/**
 *
 */
::mrta_vc::Agent unifei::expertinos::mrta_vc::agents::VoiceCommander::toMsg() 
{
	::mrta_vc::Agent voice_commander_msg = Agent::toMsg();
	voice_commander_msg.login_name = login_name_;
	//voice_commander_msg.computer = computer_.toMsg();
	return voice_commander_msg;
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
