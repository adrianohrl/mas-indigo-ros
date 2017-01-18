/**
 *  This source file implements the MachineController class.
 *
 *  Version: 1.4.0
 *  Created on: 11/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief MachineController::MachineController
 * @param nh
 */
MachineController::MachineController(ros::NodeHandle* nh)
    : nh_(nh), user_(NULL), task_(NULL), current_(NULL)
{
  states_.push_back(new S0InitialState(this));
  states_.push_back(new S1TaskVerificationState(this));
  states_.push_back(new S2TaskVerificationState(this));
  states_.push_back(new S3TaskVerificationState(this));
  states_.push_back(new S4SenderVerificationState(this));
  states_.push_back(new S5SenderVerificationState(this));
  states_.push_back(new S6ReceiverVerificationState(this));
  states_.push_back(new S7PriorityVerificationState(this));
  states_.push_back(new S8DeadlineVerificationState(this));
  states_.push_back(new S9FinalState(this));
  reset();
}

/**
 * @brief MachineController::~MachineController
 */
MachineController::~MachineController()
{
  current_ = NULL;
  for (int i(0); i < states_.size(); i++)
  {
    if (states_[i])
    {
      delete states_[i];
      states_[i] = NULL;
    }
  }
}

/**
 * @brief MachineController::getNodeHandle
 * @return
 */
ros::NodeHandle* MachineController::getNodeHandle() const { return nh_; }

/**
 * @brief MachineController::getQuestion
 * @return
 */
std::string MachineController::getQuestion() const
{
  return current_->getQuestion();
}

/**
 * @brief MachineController::getMessage
 * @return
 */
std::string MachineController::getMessage() const
{
  return current_->getMessage();
}

/**
 * @brief MachineController::isFinalState
 * @return
 */
bool MachineController::isFinalState() const
{
  return current_->isFinalState();
}

/**
 * @brief MachineController::getTask
 * @return
 */
mas::tasks::Task* MachineController::getTask() const { return task_; }

/**
 * @brief MachineController::getUser
 * @return
 */
mas::agents::User* MachineController::getUser() const { return user_; }

/**
 * @brief MachineController::setNext
 * @param state
 */
bool MachineController::setNext(unsigned int state)
{
  if (state > states_.size())
  {
    throw new utilities::Exception("Invalid state number");
  }
  current_ = states_[state];
  return current_;
}

/**
 * @brief MachineController::setUser
 * @param user
 */
void MachineController::setUser(mas::agents::User* user)
{
  if (user_)
  {
    delete user_;
    user_ = NULL;
  }
  user_ = user;
  if (task_)
  {
    task_->setUser(user_);
  }
}

/**
 * @brief MachineController::setUser
 * @param msg
 */
void MachineController::setUser(const mas_msgs::Agent &msg)
{
  if (!user_ || user_ && *user_ != msg)
  {
    setUser(new mas::agents::User(msg));
  }
}

/**
 * @brief MachineController::setUser
 * @param msg
 */
void MachineController::setUser(const mas_msgs::Agent::ConstPtr &msg)
{
  if (!user_ || user_ && *user_ != msg)
  {
    setUser(new mas::agents::User(msg));
  }
}

/**
 * @brief MachineController::setTask
 * @param task
 */
void MachineController::setTask(mas::tasks::Task* task)
{
  if (task_)
  {
    delete task_;
  }
  task_ = task;
  if (task_)
  {
    task_->setUser(user_);
  }
}

/**
 * @brief MachineController::setTaskSender
 * @param sender
 */
void MachineController::setTaskSender(mas::agents::Person* sender)
{
  task_->setSender(sender);
}

/**
 * @brief MachineController::setTaskReceiver
 * @param receiver
 */
void MachineController::setTaskReceiver(mas::agents::Person* receiver)
{
  task_->setReceiver(receiver);
}

/**
 * @brief MachineController::setTaskPriority
 * @param priority
 */
void MachineController::setTaskPriority(mas::tasks::TaskPriorityEnum priority)
{
  task_->setPriority(priority);
}

/**
 * @brief MachineController::setTaskDeadline
 * @param deadline
 */
void MachineController::setTaskDeadline(const ros::Time& deadline)
{
  task_->setDeadline(deadline);
}

/**
 * @brief MachineController::setTaskDeadline
 * @param duration
 */
void MachineController::setTaskDeadline(const ros::Duration& duration)
{
  task_->setDeadline(duration);
}

/**
 * @brief MachineController::process
 * @param answer
 * @return
 */
bool MachineController::process(std::string answer)
{
  if (answer == "cancel" || answer == "abort")
  {
    reset();
    return true;
  }
  return current_->process(answer);
}

/**
 * @brief MachineController::reset
 */
void MachineController::reset()
{
  current_ = states_[0];
  task_ = NULL;
}

/**
 * @brief MachineController::str
 * @return
 */
std::string MachineController::str() const { return current_->str(); }

/**
 * @brief MachineController::c_str
 * @return
 */
const char* MachineController::c_str() const { return str().c_str(); }
}
