/**
 *  SystemDatabaseInterfaceNode.h
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Christiano Henrique Rezende (c.h.rezende@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_DATABASE_INTERFACE_NODE_H_
#define SYSTEM_DATABASE_INTERFACE_NODE_H_

#include <ros/ros.h>

namespace mrta_vc 
{

	class SystemDatabaseInterfaceNode 
	{

	public:

		/** Construtors */
		SystemDatabaseInterfaceNode(ros::NodeHandle nh);
		/** Destrutor */
		~SystemDatabaseInterfaceNode();

		/** métodos publicos relacionados ao gerenciamento do nó */
		void spin();

	private:
	
		/** atributos privados relacionados ao nó */
		ros::NodeHandle nh_;

	};

}

#endif /* SYSTEM_DATABASE_INTERFACE_NODE_H_ */
