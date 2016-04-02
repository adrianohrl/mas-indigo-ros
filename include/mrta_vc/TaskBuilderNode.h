/**
 *  TaskBuilderNode.h
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_NODE_H_
#define TASK_BUILDER_NODE_H_

#include <ros/ros.h>

namespace mrta_vc 
{

	class TaskBuilderNode 
	{

	public:

		/** Construtors */
		TaskBuilderNode(ros::NodeHandle nh);
		/** Destrutor */
		~TaskBuilderNode();

		/** métodos publicos relacionados ao gerenciamento do nó */
		void spin();

	private:
	
		/** atributos privados relacionados ao nó */
		ros::NodeHandle nh_;

	};

}

#endif /* TASK_BUILDER_NODE_H_ */
