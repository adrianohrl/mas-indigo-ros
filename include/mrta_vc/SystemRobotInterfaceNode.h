/**
 *  SystemRobotInterfaceNode.h
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_ROBOT_INTERFACE_NODE_H_
#define SYSTEM_ROBOT_INTERFACE_NODE_H_

#include <ros/ros.h>

namespace mrta_vc 
{

	class SystemRobotInterfaceNode 
	{

	public:

		/** Construtores */
		SystemRobotInterfaceNode(ros::NodeHandle nh);
		/** Destrutor */
		~SystemRobotInterfaceNode();

		/** métodos publicos relacionados ao gerenciamento do nó */
		void spin();

	private:
	
		/** atributos privados relacionados ao nó */
		ros::NodeHandle nh_;

	};

}

#endif /* SYSTEM_ROBOT_INTERFACE_NODE_H_ */
