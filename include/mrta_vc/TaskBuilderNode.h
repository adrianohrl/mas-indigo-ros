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
#include <std_srvs/Empty.h>
#include "mrta_vc/Task.h"
#include "mrta_vc/GetPerson.h"
#include "mrta_vc/GetUser.h"
#include "unifei/expertinos/mrta_vc/tasks/Task.h"
#include "unifei/expertinos/mrta_vc/agents/User.h"

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
    ros::Publisher task_pub_;
    ros::ServiceServer abort_srv_;
    ros::ServiceClient get_person_cli_;
    ros::ServiceClient get_user_cli_;
    unifei::expertinos::mrta_vc::tasks::Task task_;

    bool abort(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	};

}

#endif /* TASK_BUILDER_NODE_H_ */
