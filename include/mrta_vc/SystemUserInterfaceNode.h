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
#include "mrta_vc/ValidatePassword.h"
#include "unifei/expertinos/mrta_vc/agents/VoiceCommander.h"

namespace mrta_vc 
{

  class SystemUserInterfaceNode : public unifei::expertinos::mrta_vc::agents::VoiceCommander
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
		ros::ServiceClient validate_cli_;
    bool logged_;
		
		void beaconTimerCallback(const ros::TimerEvent& event);
		void login(std::string login_name, std::string password);
		void logout();
		void setComputerUp();


	};

}

#endif /* SYSTEM_USER_INTERFACE_NODE_H_ */
