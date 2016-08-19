/**
 *  TaskBuilderNode.h
 *
 *  Version: 1.2.4
 *  Created on: 01/04/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_NODE_H_
#define TASK_BUILDER_NODE_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <mas_srvs/GetPerson.h>
#include <mas_srvs/GetUser.h>
#include <mas_srvs/SetUser.h>
#include <mas/tasks/Task.h>
#include <mas/tasks/task_builder/MachineController.h>

namespace mas_task_builder 
{

	class TaskBuilderNode : public mas::tasks::task_builder::MachineController
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
		bool setUser(mas_srvs::SetUser::Request& request, mas_srvs::SetUser::Response& response);
		void questionsTimerCallback(const ros::TimerEvent& event);
		void publishQuestionAndMessage();
		bool isUserLogged();

	};

}

#endif /* TASK_BUILDER_NODE_H_ */
