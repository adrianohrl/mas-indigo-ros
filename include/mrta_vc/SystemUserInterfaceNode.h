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
#include "mrta_vc/ValidatePassword.h"
#include "unifei/expertinos/mrta_vc/agents/User.h"

namespace mrta_vc 
{

  class SystemUserInterfaceNode : public unifei::expertinos::mrta_vc::agents::User
	{

	public:
		/** Construtors */
		SystemUserInterfaceNode(ros::NodeHandle nh);
		/** Destrutor */
		~SystemUserInterfaceNode();

		/** métodos publicos relacionados ao gerenciamento do nó */
		void spin();

	private:
		/** atributos privados relacionados ao nó */
		ros::NodeHandle nh_;
		ros::Timer beacon_timer_;
		ros::Publisher beacon_pub_;
		ros::Subscriber message_sub_;
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
