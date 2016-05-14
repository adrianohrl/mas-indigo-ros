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
#include <std_msgs/String.h>
#include "mrta_vc/GetPerson.h"
#include "mrta_vc/GetUser.h"
#include "mrta_vc/state_machine/MachineController.h"
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
		ros::Publisher question_pub_;
		ros::Publisher message_pub_;
		ros::Subscriber answer_sub_;
		ros::Publisher task_pub_;
    ros::ServiceServer abort_srv_;
    ros::ServiceClient get_person_cli_;
    ros::ServiceClient get_user_cli_;

		state_machine::MachineController sm_controller_;

		void answersCallback(const std_msgs::String::ConstPtr& answer_msg);
    bool abort(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	};

}

#endif /* TASK_BUILDER_NODE_H_ */
