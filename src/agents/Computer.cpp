/**
 *  Computer.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/agents/Computer.h"

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Computer::Computer(int id, std::string hostname, bool mobile, double x, double y, double theta) : Agent(id, x, y, theta)
{
	hostname_ = hostname;
	mobile_ = mobile;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Computer::Computer(int id, std::string hostname, bool mobile, geometry_msgs::Pose pose_msg) : Agent(id, pose_msg)
{
	hostname_ = hostname;
	mobile_ = mobile;	
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Computer::Computer(const ::mrta_vc::Agent::ConstPtr& computer_msg) : Agent(computer_msg)
{
	hostname_ = computer_msg->name;
	mobile_ = computer_msg->mobile;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Computer::Computer(::mrta_vc::Agent computer_msg) : Agent(computer_msg)
{
	hostname_ = computer_msg.name;
	mobile_ = computer_msg.mobile;		
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Computer::~Computer() 
{
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::Computer::getHostname() 
{
	return hostname_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Computer::isMobile() 
{
	return mobile_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::agents::Computer::getType() 
{
	return COMPUTER;
}

/**
 *
 */
::mrta_vc::Agent unifei::expertinos::mrta_vc::agents::Computer::toMsg() 
{
	::mrta_vc::Agent computer_msg = Agent::toMsg();
	computer_msg.name = hostname_;
	computer_msg.mobile = mobile_;
	return computer_msg;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Computer::equals(unifei::expertinos::mrta_vc::agents::Computer computer) 
{
	return hostname_ == computer.hostname_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Computer::operator==(const unifei::expertinos::mrta_vc::agents::Computer& computer)
{
	return hostname_ == computer.hostname_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Computer::operator=(const unifei::expertinos::mrta_vc::agents::Computer& computer) 
{
	unifei::expertinos::mrta_vc::agents::Agent::operator=(computer);
	hostname_ = computer.hostname_;
	mobile_ = computer.mobile_;
}
