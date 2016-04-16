/**
 *  SystemUserInterfaceNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemUserInterfaceNode.h"

/**
 * Constructor
 */
mrta_vc::SystemUserInterfaceNode::SystemUserInterfaceNode(ros::NodeHandle nh) : unifei::expertinos::mrta_vc::agents::VoiceCommander(), nh_(nh)
{
	beacon_timer_ = nh_.createTimer(ros::Duration(USER_BEACON_INTERVAL_DURATION), &mrta_vc::SystemUserInterfaceNode::beaconTimerCallback, this);
	beacon_pub_ = nh_.advertise<mrta_vc::Agent>("/user_beacon", 1);
	setComputerUp();
}

/**
 * Destructor
 */
mrta_vc::SystemUserInterfaceNode::~SystemUserInterfaceNode()
{
	beacon_pub_.shutdown();
}

/**
 * 
 */
void mrta_vc::SystemUserInterfaceNode::spin() 
{
	ROS_INFO("System User Interface Node is up and running!!!");
	ros::Rate loop_rate(10.0);
	while (nh_.ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/**
 * 
 */
void mrta_vc::SystemUserInterfaceNode::beaconTimerCallback(const ros::TimerEvent& event) 
{
	beacon_pub_.publish(unifei::expertinos::mrta_vc::agents::VoiceCommander::toMsg());
}

/**
 *
 */
void mrta_vc::SystemUserInterfaceNode::setComputerUp()
{
	bool setted_up;
	std::string ns = ros::this_node::getName();
	ns.append("/computer"); 
	ROS_DEBUG("********* Reading User Computer Parameters **********");
	
	int user_computer_id = 0;
	nh_.param<int>(ns + std::string("/id"), user_computer_id, 0);
	ROS_ERROR_COND(user_computer_id == 0, "Invalid user computer ID!!!");
	setted_up = user_computer_id != 0;
	
	std::string user_computer_hostname = "";
	nh_.param<std::string>(ns + std::string("/hostname"), user_computer_hostname, "");
	ROS_ERROR_COND(user_computer_hostname == "", "Invalid user computer hostname!!!");
	setted_up = setted_up && user_computer_hostname != "";
	
	bool user_computer_mobile;
	nh_.param<bool>(ns + std::string("/mobile"), user_computer_mobile, false);
	
	double user_computer_location_x, user_computer_location_y, user_computer_location_theta;
	
	ns.append("/location"); 
	nh_.param<double>(ns + std::string("/x"), user_computer_location_x, 0);
	nh_.param<double>(ns + std::string("/y"), user_computer_location_y, 0);
	nh_.param<double>(ns + std::string("/theta"), user_computer_location_theta, 0);
	
	unifei::expertinos::mrta_vc::agents::Computer user_computer(user_computer_id, user_computer_hostname, user_computer_mobile, user_computer_location_x, user_computer_location_y, user_computer_location_theta);
	unifei::expertinos::mrta_vc::agents::VoiceCommander::setComputer(user_computer);	
	if (!setted_up) 
	{
		ROS_ERROR("You must create and set a YAML file containing this machine info.");
	} 
	else 
	{
		ROS_INFO("This computer has been setted up!!!");
		ROS_INFO("This Computer Info:");
		ROS_INFO("     id: %d", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getId());
		ROS_INFO("     hostname: %s", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getHostname().c_str());
		ROS_INFO("     mobile: %s", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().isMobile() ? "true" : "false");
		ROS_INFO("     location: (%f, %f, %f)", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getLocation().getX(), unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getLocation().getY(), unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getLocation().getTheta());
	}
}
