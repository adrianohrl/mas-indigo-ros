/**
 *  This header file defines the MachineController class.
 *
 *  Version: 1.4.0
 *  Created on: 10/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_MACHINE_CONTROLLER_H_
#define _TASK_BUILDER_MACHINE_CONTROLLER_H_

#include <string>
#include <ros/ros.h>
#include <utilities/exception.h>
#include <mas/tasks/task.h>
#include <mas/agents/user.h>
#include "task_builder/abstract_state.h"
#include "task_builder/s0_initial_state.h"
#include "task_builder/s1_task_verification_state.h"
#include "task_builder/s2_task_verification_state.h"
#include "task_builder/s3_task_verification_state.h"
#include "task_builder/s4_sender_verification_state.h"
#include "task_builder/s5_sender_verification_state.h"
#include "task_builder/s6_receiver_verification_state.h"
#include "task_builder/s7_priority_verification_state.h"
#include "task_builder/s8_deadline_verification_state.h"
#include "task_builder/s9_final_state.h"

namespace task_builder
{
class MachineController
{
public:
  MachineController(ros::NodeHandle* nh);
  virtual ~MachineController();
  ros::NodeHandle* getNodeHandle() const;
  std::string getQuestion() const;
  std::string getMessage() const;
  bool isFinalState() const;
  mas::tasks::Task* getTask() const;
  mas::agents::User* getUser() const;
  bool setNext(unsigned int state);
  void setTask(mas::tasks::Task* task);
  void setTaskSender(mas::agents::Person* sender);
  void setTaskReceiver(mas::agents::Person* receiver);
  void setTaskPriority(mas::tasks::TaskPriorityEnum priority);
  void setTaskDeadline(const ros::Time& deadline);
  void setTaskDeadline(const ros::Duration& duration);
  bool process(std::string answer);
  void reset();
  void setUser(mas::agents::User* user);
  void setUser(const mas_msgs::Agent& msg);
  void setUser(const mas_msgs::Agent::ConstPtr& msg);
  virtual std::string str() const;
  const char* c_str() const;

private:
  ros::NodeHandle* nh_;
  mas::tasks::Task* task_;
  mas::agents::User* user_;
  AbstractState* current_;
  std::vector<AbstractState*> states_;
};
}

#endif /* _TASK_BUILDER_MACHINE_CONTROLLER_H_ */
