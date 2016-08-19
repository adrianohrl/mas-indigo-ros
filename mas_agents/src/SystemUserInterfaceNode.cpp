/**
 *  SystemUserInterfaceNode.cpp
 *
 *  Version: 1.2.4
 *  Created on: 26/03/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas_agents/SystemUserInterfaceNode.h"

using typename mas::agents::Computer;
using typename mas::agents::HierarchyLevels;
using typename mas::agents::Person;
using typename mas::agents::User;
using typename mas::tasks::Allocation;

namespace mas_agents
{

	/**
	 * Constructor
	 */
	SystemUserInterfaceNode::SystemUserInterfaceNode(ros::NodeHandle nh) : User()
	{
		beacon_timer_ = nh_.createTimer(ros::Duration(USER_BEACON_INTERVAL_DURATION), &SystemUserInterfaceNode::beaconTimerCallback, this);
		beacon_pub_ = nh_.advertise<mas_msgs::Agent>("/users", 1);
		allocation_pub_ = nh_.advertise<mas_msgs::Agent>("/finished_allocations", 1);
		message_sub_ = nh_.subscribe("messages", 1, &SystemUserInterfaceNode::messagesCallback, this);
		allocation_sub_ = nh_.subscribe("/running_allocations", 1, &SystemUserInterfaceNode::allocationsCallback, this);
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
	SystemUserInterfaceNode::~SystemUserInterfaceNode()
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
	void SystemUserInterfaceNode::spin() 
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
	void SystemUserInterfaceNode::beaconTimerCallback(const ros::TimerEvent& event) 
	{
		if (logged_)
		{
			beacon_pub_.publish(User::toMsg());
		}
	}

	/**
	 *
	 */
	void SystemUserInterfaceNode::messagesCallback(const std_msgs::String::ConstPtr& message_msg)
	{
		//ROS_INFO("[MESSAGE]: %s", message_msg->data.c_str());
	}

	/**
	 *
	 */
	void SystemUserInterfaceNode::allocationsCallback(const mas_msgs::Allocation::ConstPtr& allocation_msg)
	{
		if (logged_ && !hasAssignedAnyTask())
		{
			return;
		}
		Allocation allocation(allocation_msg);
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
	void SystemUserInterfaceNode::login(std::string login_name, std::string password)
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
		User user(validate_srv.response.user);
		user.setComputer(User::getComputer());
		User::operator=(user);
		ROS_DEBUG("Login succeeded: %s", validate_srv.response.message.c_str());
		ROS_DEBUG("%s", User::toString().c_str());
		mas_srvs::SetUser set_user_srv;
		set_user_srv.request.logged = true;
		set_user_srv.request.user = User::toMsg();
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
		ROS_INFO("     id: %d", User::getId());
		ROS_INFO("     name: %s", User::getName().c_str());
		ROS_INFO("     hierarchy_level: %s", HierarchyLevels::toString(Person::getHierarchyLevel()).c_str());
		ROS_INFO("     login_name: %s", User::getLoginName().c_str());
		ROS_INFO("     computer:");
		ROS_INFO("          id: %d", User::getComputer().getId());
		ROS_INFO("          hostname: %s", User::getComputer().getHostname().c_str());
		ROS_INFO("          mobile: %s", User::getComputer().isMobile() ? "true" : "false");
		ROS_INFO("          location: (%f, %f, %f)", User::getComputer().getLocation().getX(), User::getComputer().getLocation().getY(), User::getComputer().getLocation().getTheta());
		ROS_INFO("     location: (%f, %f, %f)", User::getLocation().getX(), User::getLocation().getY(), User::getLocation().getTheta());
		ROS_INFO("User is logged in!!!");
	}

	/**
	 *
	 */
	void SystemUserInterfaceNode::logout()
	{
		ROS_DEBUG("%s has been logged out!!!", User::getLoginName().c_str());
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
	void SystemUserInterfaceNode::setComputerUp()
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
		User::setComputer(Computer(srv.response.computer));
		ROS_INFO("This computer has been setted up!!!");
		ROS_DEBUG("Computer setting up succeeded: %s", srv.response.message.c_str());
		ROS_DEBUG("%s", User::getComputer().toString().c_str());
	}

	/**
	 *
	 */
	bool SystemUserInterfaceNode::hasAssignedAnyTask()
	{
		return !allocations_.empty();
	}
	
}
