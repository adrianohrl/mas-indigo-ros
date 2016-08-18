/**
 *  SystemUserInterfaceNode.cpp
 *
 *  Version: 1.2.2
 *  Created on: 26/03/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemUserInterfaceNode.h"

/**
 * Constructor
 */
mrta_vc::SystemUserInterfaceNode::SystemUserInterfaceNode(ros::NodeHandle nh) : unifei::expertinos::mrta_vc::agents::User()
{
	beacon_timer_ = nh_.createTimer(ros::Duration(USER_BEACON_INTERVAL_DURATION), &mrta_vc::SystemUserInterfaceNode::beaconTimerCallback, this);
	beacon_pub_ = nh_.advertise<mas_msgs::Agent>("/users", 1);
	allocation_pub_ = nh_.advertise<mas_msgs::Agent>("/finished_allocations", 1);
	message_sub_ = nh_.subscribe("messages", 1, &mrta_vc::SystemUserInterfaceNode::messagesCallback, this);
	allocation_sub_ = nh_.subscribe("/running_allocations", 1, &mrta_vc::SystemUserInterfaceNode::allocationsCallback, this);
	get_computer_cli_ = nh_.serviceClient<mas_srvs::GetComputer>("/get_computer");
	set_user_cli_ = nh_.serviceClient<mas_srvs::SetUser>("set_user");
	validate_cli_ = nh_.serviceClient<mas_srvs::ValidatePassword>("/validate_password");
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
	allocation_pub_.shutdown();
	message_sub_.shutdown();
	allocation_sub_.shutdown();
	get_computer_cli_.shutdown();
	set_user_cli_.shutdown();
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
		beacon_pub_.publish(unifei::expertinos::mrta_vc::agents::User::toMsg());
	}
}

/**
 *
 */
void mrta_vc::SystemUserInterfaceNode::messagesCallback(const std_msgs::String::ConstPtr& message_msg)
{
	//ROS_INFO("[MESSAGE]: %s", message_msg->data.c_str());
}

/**
 *
 */
void mrta_vc::SystemUserInterfaceNode::allocationsCallback(const mas_msgs::Allocation::ConstPtr& allocation_msg)
{
	if (logged_ && !hasAssignedAnyTask())
	{
		return;
	}
	unifei::expertinos::mrta_vc::tasks::Allocation allocation(allocation_msg);
	if (allocation.isInvolved(*this))
	{
		ROS_INFO("I'm involved in: %s", allocation.toString().c_str());
		allocations_.push_back(allocation);
		//allocation_.start();
		//allocation_pub_.publish(allocation_.toMsg());
	}
}

/**
 *
 */
void mrta_vc::SystemUserInterfaceNode::login(std::string login_name, std::string password)
{
	logged_ = false;
	mas_srvs::ValidatePassword validate_srv;
	validate_srv.request.login_name = login_name;
	validate_srv.request.password = password;
	if (!validate_cli_.call(validate_srv) || !validate_srv.response.valid)
	{
		ROS_ERROR("Login failure: %s", validate_srv.response.message.c_str());
		return;
	}
	unifei::expertinos::mrta_vc::agents::User user(validate_srv.response.user);
	user.setComputer(unifei::expertinos::mrta_vc::agents::User::getComputer());
	unifei::expertinos::mrta_vc::agents::User::operator=(user);
	ROS_DEBUG("Login succeeded: %s", validate_srv.response.message.c_str());
	ROS_DEBUG("%s", unifei::expertinos::mrta_vc::agents::User::toString().c_str());
	mas_srvs::SetUser set_user_srv;
	set_user_srv.request.logged = true;
	set_user_srv.request.user = unifei::expertinos::mrta_vc::agents::User::toMsg();
	if (!set_user_cli_.waitForExistence(ros::Duration(5)))
	{
		ROS_ERROR("You must run this user task_builder_node at first!!!");
		return;
	}
	if (!set_user_cli_.call(set_user_srv))
	{
		ROS_ERROR("Unable to log user in!!!");
		return;
	}
	logged_ = true;
	ROS_INFO("User Info:");
	ROS_INFO("     id: %d", unifei::expertinos::mrta_vc::agents::User::getId());
	ROS_INFO("     name: %s", unifei::expertinos::mrta_vc::agents::User::getName().c_str());
	ROS_INFO("     hierarchy_level: %s", unifei::expertinos::mrta_vc::agents::HierarchyLevels::toString(unifei::expertinos::mrta_vc::agents::Person::getHierarchyLevel()).c_str());
	ROS_INFO("     login_name: %s", unifei::expertinos::mrta_vc::agents::User::getLoginName().c_str());
	ROS_INFO("     computer:");
	ROS_INFO("          id: %d", unifei::expertinos::mrta_vc::agents::User::getComputer().getId());
	ROS_INFO("          hostname: %s", unifei::expertinos::mrta_vc::agents::User::getComputer().getHostname().c_str());
	ROS_INFO("          mobile: %s", unifei::expertinos::mrta_vc::agents::User::getComputer().isMobile() ? "true" : "false");
	ROS_INFO("          location: (%f, %f, %f)", unifei::expertinos::mrta_vc::agents::User::getComputer().getLocation().getX(), unifei::expertinos::mrta_vc::agents::User::getComputer().getLocation().getY(), unifei::expertinos::mrta_vc::agents::User::getComputer().getLocation().getTheta());
	ROS_INFO("     location: (%f, %f, %f)", unifei::expertinos::mrta_vc::agents::User::getLocation().getX(), unifei::expertinos::mrta_vc::agents::User::getLocation().getY(), unifei::expertinos::mrta_vc::agents::User::getLocation().getTheta());
	ROS_INFO("User is logged in!!!");
}

/**
 *
 */
void mrta_vc::SystemUserInterfaceNode::logout()
{
	ROS_DEBUG("%s has been logged out!!!", unifei::expertinos::mrta_vc::agents::User::getLoginName().c_str());
	mas_srvs::SetUser srv;
	srv.request.logged = false;
	srv.request.user = mas_msgs::Agent();
	if (!set_user_cli_.call(srv))
	{
		ROS_ERROR("Unable to log user out!!!");
		return;
	}
	logged_ = false;
}

/**
 *
 */
void mrta_vc::SystemUserInterfaceNode::setComputerUp()
{
	std::string hostname;
	nh_.param<std::string>(ros::this_node::getName() + std::string("/computer/hostname"), hostname, "");
	if (hostname == "")
	{
		ROS_ERROR("You must create and set a YAML file containing this machine info.");
		nh_.shutdown();
		return;
	}
	mas_srvs::GetComputer srv;
	srv.request.hostname = hostname;
	if (!get_computer_cli_.call(srv) || !srv.response.valid)
	{
		ROS_ERROR("Computer setting up failure: %s", srv.response.message.c_str());
		nh_.shutdown();
		return;
	}
	unifei::expertinos::mrta_vc::agents::User::setComputer(unifei::expertinos::mrta_vc::agents::Computer(srv.response.computer));
	ROS_INFO("This computer has been setted up!!!");
	ROS_DEBUG("Computer setting up succeeded: %s", srv.response.message.c_str());
	ROS_DEBUG("%s", unifei::expertinos::mrta_vc::agents::User::getComputer().toString().c_str());
}

/**
 *
 */
bool mrta_vc::SystemUserInterfaceNode::hasAssignedAnyTask()
{
	return !allocations_.empty();
}
