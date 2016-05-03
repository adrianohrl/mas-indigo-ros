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
  beacon_pub_ = nh_.advertise<mrta_vc::Agent>("/users", 1);
	validate_cli_ = nh_.serviceClient<mrta_vc::ValidatePassword>("/validate_password");
	setComputerUp();
	logged_ = false;
	std::string ns = ros::this_node::getName();
	if (nh_.hasParam(ns + std::string("/login_name")) && nh_.hasParam(ns + std::string("/password")))
	{
		std::string login_name;
		std::string password;
		nh_.param<std::string>(ns + std::string("/login_name"), login_name, "");
		nh_.param<std::string>(ns + std::string("/password"), password, "");
		login(login_name, password);
	}
}

/**
 * Destructor
 */
mrta_vc::SystemUserInterfaceNode::~SystemUserInterfaceNode()
{
  logout();
  beacon_timer_.stop();
  beacon_pub_.shutdown();
  validate_cli_.shutdown();
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
	if (logged_)
	{
		beacon_pub_.publish(unifei::expertinos::mrta_vc::agents::VoiceCommander::toMsg());	
	}
}

/**
 * 
 */
void mrta_vc::SystemUserInterfaceNode::login(std::string login_name, std::string password)
{
	logged_ = false;
	mrta_vc::ValidatePassword srv;
	srv.request.login_name = login_name;
	srv.request.password = password;
	if (validate_cli_.call(srv) && srv.response.valid) 
	{
		unifei::expertinos::mrta_vc::agents::VoiceCommander::operator=(srv.response.voice_commander);
		unifei::expertinos::mrta_vc::agents::VoiceCommander::setComputer(unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer());
		ROS_DEBUG("Login succeeded: %s", srv.response.message.c_str());
		ROS_INFO("User is logged in!!!");
		ROS_INFO("User Info:");
		ROS_INFO("     id: %d", unifei::expertinos::mrta_vc::agents::VoiceCommander::getId());
		ROS_INFO("     name: %s", unifei::expertinos::mrta_vc::agents::VoiceCommander::getName().c_str());
		ROS_INFO("     hierarchy_level: %s", unifei::expertinos::mrta_vc::agents::HierarchyLevels::toString(unifei::expertinos::mrta_vc::agents::Person::getHierarchyLevel()).c_str());
		ROS_INFO("     login_name: %s", unifei::expertinos::mrta_vc::agents::VoiceCommander::getLoginName().c_str());
		ROS_INFO("     computer:");
		ROS_INFO("          id: %d", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getId());
		ROS_INFO("          hostname: %s", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getHostname().c_str());
		ROS_INFO("          mobile: %s", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().isMobile() ? "true" : "false");
		ROS_INFO("          location: (%f, %f, %f)", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getLocation().getX(), unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getLocation().getY(), unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getLocation().getTheta());
		ROS_INFO("     location: (%f, %f, %f)", unifei::expertinos::mrta_vc::agents::VoiceCommander::getLocation().getX(), unifei::expertinos::mrta_vc::agents::VoiceCommander::getLocation().getY(), unifei::expertinos::mrta_vc::agents::VoiceCommander::getLocation().getTheta());
		beacon_pub_.publish(unifei::expertinos::mrta_vc::agents::VoiceCommander::toMsg());
		logged_ = true;
	}
	else
	{
		ROS_ERROR("Login failure: %s", srv.response.message.c_str());
	}
}

/**
 * 
 */
void mrta_vc::SystemUserInterfaceNode::logout()
{
	ROS_DEBUG("%s has been logged out!!!", unifei::expertinos::mrta_vc::agents::VoiceCommander::getLoginName().c_str());
	logged_ = false;
}

/**
 *
 */
void mrta_vc::SystemUserInterfaceNode::setComputerUp()
{
	ROS_DEBUG("********* Reading User Computer Parameters **********");
  bool setted_up;
  std::string ns = ros::this_node::getName();
  ns.append("/computer");
	
	int user_computer_id = 0;
  /*nh_.param<int>(ns + std::string("/id"), user_computer_id, 0);
	ROS_ERROR_COND(user_computer_id == 0, "Invalid user computer ID!!!");
  setted_up = user_computer_id != 0;*/
	
  std::string user_computer_hostname;
	nh_.param<std::string>(ns + std::string("/hostname"), user_computer_hostname, "");
	ROS_ERROR_COND(user_computer_hostname == "", "Invalid user computer hostname!!!");
  setted_up = user_computer_hostname != "";
	
	bool user_computer_mobile;
	nh_.param<bool>(ns + std::string("/mobile"), user_computer_mobile, false);

  ns.append("/location");
	double user_computer_location_x, user_computer_location_y, user_computer_location_theta;
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
		ROS_INFO("Computer Info:");
		ROS_INFO("     id: %d", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getId());
		ROS_INFO("     hostname: %s", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getHostname().c_str());
		ROS_INFO("     mobile: %s", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().isMobile() ? "true" : "false");
		ROS_INFO("     location: (%f, %f, %f)", unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getLocation().getX(), unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getLocation().getY(), unifei::expertinos::mrta_vc::agents::VoiceCommander::getComputer().getLocation().getTheta());
	}
}
