/**
 *  SystemUserInterfaceNode.h
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_USER_INTERFACE_NODE_H_
#define SYSTEM_USER_INTERFACE_NODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "mrta_vc/GetComputer.h"
#include "mrta_vc/SetUser.h"
#include "mrta_vc/ValidatePassword.h"
#include "unifei/expertinos/mrta_vc/agents/User.h"

namespace mrta_vc 
{

  class SystemUserInterfaceNode : public unifei::expertinos::mrta_vc::agents::User
	{

	public:
		SystemUserInterfaceNode(ros::NodeHandle nh);
		virtual ~SystemUserInterfaceNode();

		void spin();

	private:
		ros::NodeHandle nh_;
		ros::Timer beacon_timer_;
		ros::Publisher beacon_pub_;
		ros::Subscriber message_sub_;
		ros::ServiceClient get_computer_cli_;
		ros::ServiceClient set_user_cli_;
		ros::ServiceClient validate_cli_;
    bool logged_;
		
		void beaconTimerCallback(const ros::TimerEvent& event);
		void messagesCallback(const std_msgs::String::ConstPtr& message_msg);
		void login(std::string login_name, std::string password);
		void logout();
		void setComputerUp();


	};

}

#endif /* SYSTEM_USER_INTERFACE_NODE_H_ */
