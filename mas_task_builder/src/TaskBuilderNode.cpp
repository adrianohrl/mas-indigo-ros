/**
 *  TaskBuilderNode.cpp
 *
 *  Version: 1.2.4
 *  Created on: 01/04/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas_task_builder/TaskBuilderNode.h"

using typename mas::agents::User;
using typename mas::tasks::task_builder::MachineController;

namespace mas_task_builder
{

	/**
	 * Constructor
	 */
	TaskBuilderNode::TaskBuilderNode(ros::NodeHandle nh) : MachineController(nh)
	{
		questions_timer_ = nh.createTimer(ros::Duration(2), &TaskBuilderNode::questionsTimerCallback, this);
		question_pub_ = nh.advertise<std_msgs::String>("questions", 1);
		message_pub_ = nh.advertise<std_msgs::String>("messages", 1);
		answer_sub_ = nh.subscribe("answers", 1, &TaskBuilderNode::answersCallback, this);
		task_pub_ = nh.advertise<mas_msgs::Task>("/tasks", 1);
		abort_srv_ = nh.advertiseService("abort", &TaskBuilderNode::abort, this);
		set_user_srv_ = nh.advertiseService("set_user", &TaskBuilderNode::setUser, this);
		get_person_cli_ = nh.serviceClient<mas_srvs::GetPerson>("/get_person");
		get_user_cli_ = nh.serviceClient<mas_srvs::GetUser>("/get_user");
	}

	/**
	 * Destructor
	 */
	TaskBuilderNode::~TaskBuilderNode()
	{
		questions_timer_.stop();
		question_pub_.shutdown();
		message_pub_.shutdown();
		answer_sub_.shutdown();
		task_pub_.shutdown();
		abort_srv_.shutdown();
		set_user_srv_.shutdown();
		get_person_cli_.shutdown();
		get_user_cli_.shutdown();
	}

	/**
	 *
	 */
	void TaskBuilderNode::spin() 
	{
		ROS_INFO("Task Builder Node is up and running!!!");
		ros::Rate loop_rate(10.0);
		while (MachineController::getNodeHandle().ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	/**
	 *
	 */
	bool TaskBuilderNode::abort(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
	{
		MachineController::reset();
		publishQuestionAndMessage();
		return true;
	}

	/**
	 *
	 */
	bool TaskBuilderNode::setUser(mas_srvs::SetUser::Request& request, mas_srvs::SetUser::Response& response)
	{
		User user(request.user);
		if (user.getLoginName() == "")
		{
			return false;
		}
		MachineController::setUser(user);
		return true;
	}

	/**
	 *
	 */
	void TaskBuilderNode::answersCallback(const std_msgs::String::ConstPtr& answer_msg)
	{
		if (!isUserLogged())
		{
			ROS_DEBUG("[TASK_BUILDER] There is no user logged yet!!!");
			return;
		}
		if (!MachineController::process(answer_msg->data))
		{
			ROS_DEBUG("[TASK_BUILDER] Invalid answer!!!");
			return;
		}
		publishQuestionAndMessage();
		if (MachineController::isFinalState())
		{
			ROS_DEBUG("[TASK_BUILDER] task: %s", MachineController::getTask().toString().c_str());
			task_pub_.publish(MachineController::getTask().toMsg());
			MachineController::reset();
			ROS_DEBUG("----------------------------------------------");
		}
	}

	/**
	 * PARA TESTES!!!!
	 */
	void TaskBuilderNode::questionsTimerCallback(const ros::TimerEvent& event)
	{
		publishQuestionAndMessage();
	}

	/**
	 *
	 */
	void TaskBuilderNode::publishQuestionAndMessage()
	{
		std_msgs::String question_msg;
		question_msg.data = MachineController::getQuestion();
		ROS_DEBUG("[TASK_BUILDER_QUESTION] %s", question_msg.data.c_str());
		question_pub_.publish(question_msg);
		std_msgs::String message_msg;
		message_msg.data = MachineController::getMessage();
		ROS_DEBUG("[TASK_BUILDER_MESSAGE] %s", message_msg.data.c_str());
		message_pub_.publish(message_msg);
	}

	/**
	 *
	 */
	bool TaskBuilderNode::isUserLogged()
	{
		return MachineController::getUser().getLoginName() != "";
	}
	
}
