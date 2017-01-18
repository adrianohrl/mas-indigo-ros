/**
 *  This source file implements the TaskBuilderNode class, which is based on the
 *ROSNode helper class. It controls the task_builder_node.
 *
 *  Version: 1.4.0
 *  Created on: 01/04/2016
 *  Modified on: 03/01/2017
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/task_builder_node.h"

namespace task_builder
{

/**
 * @brief TaskBuilderNode::TaskBuilderNode
 * @param nh
 */
TaskBuilderNode::TaskBuilderNode(ros::NodeHandle* nh)
    : ROSNode(nh, 10), builder_(new MachineController(nh))
{
  question_pub_ = nh->advertise<std_msgs::String>("questions", 1);
  message_pub_ = nh->advertise<std_msgs::String>("messages", 1);
  answer_sub_ =
      nh->subscribe("answers", 1, &TaskBuilderNode::answersCallback, this);
  task_pub_ = nh->advertise<mas_msgs::Task>("/tasks", 1);
  abort_srv_ = nh->advertiseService("abort", &TaskBuilderNode::abort, this);
  get_user_cli_ = nh->serviceClient<mas_srvs::GetUser>("get_user");
}

/**
 * @brief TaskBuilderNode::~TaskBuilderNode
 */
TaskBuilderNode::~TaskBuilderNode()
{
  question_pub_.shutdown();
  message_pub_.shutdown();
  answer_sub_.shutdown();
  task_pub_.shutdown();
  abort_srv_.shutdown();
  get_user_cli_.shutdown();
  if (builder_)
  {
    delete builder_;
    builder_ = NULL;
  }
}

/**
 * @brief TaskBuilderNode::controlLoop
 */
void TaskBuilderNode::controlLoop()
{
  mas_srvs::GetUser srv;
  if (get_user_cli_.call(srv) && srv.response.logged)
  {
    builder_->setUser(srv.response.user);
  }
  if (isUserLogged())
  {
    publishQuestionAndMessage();
  }
}

/**
 * @brief TaskBuilderNode::abort
 * @param request
 * @param response
 * @return
 */
bool TaskBuilderNode::abort(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response)
{
  return abort();
}

/**
 * @brief TaskBuilderNode::abort
 * @return
 */
bool TaskBuilderNode::abort()
{
  builder_->reset();
  publishQuestionAndMessage();
  return true;
}

/**
 * @brief TaskBuilderNode::answersCallback
 * @param answer_msg
 */
void TaskBuilderNode::answersCallback(
    const std_msgs::String::ConstPtr& answer_msg)
{
  if (!isUserLogged())
  {
    ROS_DEBUG("[TASK_BUILDER] There is no user logged yet!!!");
    return;
  }
  if (!builder_->process(answer_msg->data))
  {
    ROS_DEBUG("[TASK_BUILDER] Invalid answer!!!");
    return;
  }
  publishQuestionAndMessage();
  if (builder_->isFinalState())
  {
    mas::tasks::Task* task = builder_->getTask();
    ROS_DEBUG("[TASK_BUILDER] task: %s", task->c_str());
    task_pub_.publish(task->to_msg());
    builder_->reset();
    ROS_DEBUG("----------------------------------------------");
  }
}

/**
 * @brief TaskBuilderNode::publishQuestionAndMessage
 */
void TaskBuilderNode::publishQuestionAndMessage()
{
  std_msgs::String question_msg;
  question_msg.data = builder_->getQuestion();
  ROS_DEBUG("[TASK_BUILDER_QUESTION] %s", question_msg.data.c_str());
  question_pub_.publish(question_msg);
  std_msgs::String message_msg;
  message_msg.data = builder_->getMessage();
  ROS_DEBUG("[TASK_BUILDER_MESSAGE] %s", message_msg.data.c_str());
  message_pub_.publish(message_msg);
}

/**
 * @brief TaskBuilderNode::isUserLogged
 * @return
 */
bool TaskBuilderNode::isUserLogged()
{
  return isUserLogged(builder_->getUser());
}

/**
 * @brief TaskBuilderNode::isUserLogged
 * @return
 */
bool TaskBuilderNode::isUserLogged(mas::agents::User* user)
{
  return user && !user->getLoginName().empty();
}
}
