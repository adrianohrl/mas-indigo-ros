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

namespace mrta_vc 
{

	class SystemUserInterfaceNode 
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

	};

}

#endif /* SYSTEM_USER_INTERFACE_NODE_H_ */
