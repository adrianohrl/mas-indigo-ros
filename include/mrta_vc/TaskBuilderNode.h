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
#include "mrta_vc/SetUser.h"
#include "mrta_vc/state_machine/MachineController.h"
#include "unifei/expertinos/mrta_vc/tasks/Task.h"

namespace mrta_vc 
{

	class TaskBuilderNode : public mrta_vc::state_machine::MachineController
	{

	public:
		TaskBuilderNode(ros::NodeHandle nh);
		virtual ~TaskBuilderNode();

		void spin();

	private:
		ros::Timer questions_timer_;
		ros::Publisher question_pub_;
		ros::Publisher message_pub_;
		ros::Subscriber answer_sub_;
		ros::Publisher task_pub_;
		ros::ServiceServer abort_srv_;
		ros::ServiceServer set_user_srv_;
		ros::ServiceClient get_person_cli_;
		ros::ServiceClient get_user_cli_;

		void answersCallback(const std_msgs::String::ConstPtr& answer_msg);
		bool abort(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool setUser(mrta_vc::SetUser::Request& request, mrta_vc::SetUser::Response& response);
		void questionsTimerCallback(const ros::TimerEvent& event);
		void publishQuestionAndMessage();
		bool isUserLogged();

	};

}

#endif /* TASK_BUILDER_NODE_H_ */
