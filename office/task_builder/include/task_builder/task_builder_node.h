/**
 *  This header file defines the TaskBuilderNode node, which is based on the
 *ROSNode helper class. It controls the task_builder_node.
 *
 *  Version: 1.4.0
 *  Created on: 01/04/2016
 *  Modified on: 03/01/2017
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_NODE_H_
#define _TASK_BUILDER_NODE_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <mas_srvs/GetUser.h>
#include <mas/tasks/task.h>
#include <utilities/ros_node.h>
#include "task_builder/machine_controller.h"

namespace task_builder
{
class TaskBuilderNode : public utilities::ROSNode
{
public:
  TaskBuilderNode(ros::NodeHandle* nh);
  virtual ~TaskBuilderNode();

private: 
  ros::Publisher question_pub_;
  ros::Publisher message_pub_;
  ros::Subscriber answer_sub_;
  ros::Publisher task_pub_;
  ros::ServiceServer abort_srv_;
  ros::ServiceClient get_user_cli_;
  MachineController* builder_;
  virtual void controlLoop();
  void answersCallback(const std_msgs::String::ConstPtr& answer_msg);
  bool abort(std_srvs::Empty::Request& request,
             std_srvs::Empty::Response& response);
  bool abort();
  void publishQuestionAndMessage();
  bool isUserLogged();
  bool isUserLogged(mas::agents::User *user);
};
}

#endif /* _TASK_BUILDER_NODE_H_ */
