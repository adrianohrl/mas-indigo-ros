/**
 *  TaskBuilderNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/TaskBuilderNode.h"

/**
 * Constructor
 */
mrta_vc::TaskBuilderNode::TaskBuilderNode(ros::NodeHandle nh) : mrta_vc::state_machine::MachineController(nh)
{
    questions_timer_ = nh.createTimer(ros::Duration(3), &mrta_vc::TaskBuilderNode::questionsTimerCallback, this);
    question_pub_ = nh.advertise<std_msgs::String>("questions", 1);
    message_pub_ = nh.advertise<std_msgs::String>("messages", 1);
    answer_sub_ = nh.subscribe("answers", 1, &mrta_vc::TaskBuilderNode::answersCallback, this);
    task_pub_ = nh.advertise<mrta_vc::Task>("/tasks", 1);
    abort_srv_ = nh.advertiseService("abort", &mrta_vc::TaskBuilderNode::abort, this);
    set_user_srv_ = nh.advertiseService("set_user", &mrta_vc::TaskBuilderNode::setUser, this);
    get_person_cli_ = nh.serviceClient<mrta_vc::GetPerson>("/get_person");
    get_user_cli_ = nh.serviceClient<mrta_vc::GetUser>("/get_user");
    counter_ = 0; //Para testes
}

/**
 * Destructor
 */
mrta_vc::TaskBuilderNode::~TaskBuilderNode()
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
void mrta_vc::TaskBuilderNode::spin() 
{
    ROS_INFO("Task Builder Node is up and running!!!");
    ros::Rate loop_rate(10.0);

    while (mrta_vc::state_machine::MachineController::getNodeHandle().ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/**
 *
 */
bool mrta_vc::TaskBuilderNode::abort(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    mrta_vc::state_machine::MachineController::reset();
    publishQuestionAndMessage();
    return true;
}

/**
 *
 */
bool mrta_vc::TaskBuilderNode::setUser(mrta_vc::SetUser::Request& request, mrta_vc::SetUser::Response& response)
{
    mrta_vc::state_machine::MachineController::setUser(unifei::expertinos::mrta_vc::agents::User(request.user));
    return true;
}

/**
 *
 */
void mrta_vc::TaskBuilderNode::answersCallback(const std_msgs::String::ConstPtr& answer_msg)
{
    ROS_DEBUG("[TASK_BUILDER] state: %s, question: %s, answer: %s", mrta_vc::state_machine::MachineController::toString().c_str(), mrta_vc::state_machine::MachineController::getQuestion().c_str(), answer_msg->data.c_str());
    if (mrta_vc::state_machine::MachineController::process(answer_msg->data))
    {
        publishQuestionAndMessage();
        if (mrta_vc::state_machine::MachineController::isFinalState())
        {
            ROS_DEBUG("[TASK_BUILDER] task: %s", mrta_vc::state_machine::MachineController::getTask().toString().c_str());
            task_pub_.publish(mrta_vc::state_machine::MachineController::getTask().toMsg());
            mrta_vc::state_machine::MachineController::reset();
            ROS_DEBUG("----------------------------------------------");
        }
    }
}

/**
 * PARA TESTES!!!!
 */
void mrta_vc::TaskBuilderNode::questionsTimerCallback(const ros::TimerEvent& event)
{
    if (counter_++ < 3)
    {
        publishQuestionAndMessage();
    }
}

/**
 *
 */
void mrta_vc::TaskBuilderNode::publishQuestionAndMessage()
{
    std_msgs::String question_msg;
    question_msg.data = mrta_vc::state_machine::MachineController::getQuestion();
    ROS_DEBUG("[TASK_BUILDER_QUESTION] %s", question_msg.data.c_str());
    question_pub_.publish(question_msg);
    std_msgs::String message_msg;
    message_msg.data = mrta_vc::state_machine::MachineController::getMessage();
    ROS_DEBUG("[TASK_BUILDER_MESSAGE] %s", message_msg.data.c_str());
    message_pub_.publish(message_msg);
}
