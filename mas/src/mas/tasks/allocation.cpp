/**
 *  This source file implements the Allocation class.
 *
 *  Version: 1.4.0
 *  Created on: 08/04/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/allocation.h"

namespace mas
{
namespace tasks
{

/**
 * @brief Allocation::Allocation
 */
Allocation::Allocation()
    : task_(NULL), state_(states::NOT_ALLOCATED), satisfaction_(satisfactions::NONE)
{
}

/**
 * @brief Allocation::Allocation
 * @param task
 * @param robots
 * @param state
 * @param satisfaction
 * @param allocation_timestamp
 * @param dispatch_timestamp
 * @param start_timestamp
 * @param end_timestamp
 */
Allocation::Allocation(Task* task, const std::vector<agents::Robot*>& robots,
                       AllocationSatisfactionEnum satisfaction,
                       const ros::Time& allocation_timestamp,
                       const ros::Time& dispatch_timestamp,
                       const ros::Time& start_timestamp,
                       const ros::Time& end_timestamp)
    : task_(task), robots_(robots), state_(states::NOT_ALLOCATED),
      satisfaction_(satisfaction), allocation_timestamp_(allocation_timestamp)
{
  setDispatchTimestamp(dispatch_timestamp);
  setStartTimestamp(start_timestamp);
  setEndTimestamp(end_timestamp);
}

/**
 * @brief Allocation::Allocation
 * @param task
 * @param robots
 */
Allocation::Allocation(Task* task, const std::vector<agents::Robot*>& robots)
    : task_(task), state_(states::NOT_ALLOCATED),
      satisfaction_(satisfactions::NONE)
{
  allocate(robots);
}

/**
 * @brief Allocation::Allocation
 * @param allocation_msg
 */
Allocation::Allocation(const mas_msgs::Allocation::ConstPtr& allocation_msg)
    : task_(new Task(allocation_msg->task))
{
  for (int i(0); i < allocation_msg->robots.size(); i++)
  {
    agents::Robot* robot = new agents::Robot(allocation_msg->robots[i]);
    robots_.push_back(robot);
  }
  state_ = AllocationStates::toEnumerated(allocation_msg->state);
  satisfaction_ =
      AllocationSatisfactions::toEnumerated(allocation_msg->satisfaction);
  allocation_timestamp_ = allocation_msg->allocation_timestamp;
  setDispatchTimestamp(allocation_msg->dispatch_timestamp);
  setStartTimestamp(allocation_msg->start_timestamp);
  setEndTimestamp(allocation_msg->end_timestamp);
}

/**
 * @brief Allocation::Allocation
 * @param allocation_msg
 */
Allocation::Allocation(const mas_msgs::Allocation& allocation_msg)
    : task_(new Task(allocation_msg.task))
{
  for (int i(0); i < allocation_msg.robots.size(); i++)
  {
    agents::Robot* robot = new agents::Robot(allocation_msg.robots[i]);
    robots_.push_back(robot);
  }
  state_ = AllocationStates::toEnumerated(allocation_msg.state);
  satisfaction_ =
      AllocationSatisfactions::toEnumerated(allocation_msg.satisfaction);
  allocation_timestamp_ = allocation_msg.allocation_timestamp;
  setDispatchTimestamp(allocation_msg.dispatch_timestamp);
  setStartTimestamp(allocation_msg.start_timestamp);
  setEndTimestamp(allocation_msg.end_timestamp);
}

/**
 * @brief Allocation::~Allocation
 */
Allocation::~Allocation() {}

/**
 * @brief Allocation::getTask
 * @return
 */
Task* Allocation::getTask() const { return task_; }

/**
 * @brief Allocation::getRobots
 * @return
 */
std::vector<agents::Robot*> Allocation::getRobots() const { return robots_; }

/**
 * @brief Allocation::getState
 * @return
 */
AllocationStateEnum Allocation::getState() const { return state_; }

/**
 * @brief Allocation::getSatisfaction
 * @return
 */
AllocationSatisfactionEnum Allocation::getSatisfaction() const
{
  return satisfaction_;
}

/**
 * @brief Allocation::getAllocationTimestamp
 * @return
 */
ros::Time Allocation::getAllocationTimestamp() const
{
  return allocation_timestamp_;
}

/**
 * @brief Allocation::getDispatchTimestamp
 * @return
 */
ros::Time Allocation::getDispatchTimestamp() const
{
  return dispatch_timestamp_;
}

/**
 * @brief Allocation::getStartTimestamp
 * @return
 */
ros::Time Allocation::getStartTimestamp() const { return start_timestamp_; }

/**
 * @brief Allocation::getEndTimestamp
 * @return
 */
ros::Time Allocation::getEndTimestamp() const { return end_timestamp_; }

/**
 * @brief Allocation::wasAllocated
 * @return
 */
bool Allocation::wasAllocated() const
{
  return state_ != states::NOT_ALLOCATED;
}

/**
 * @brief Allocation::wasDispatched
 * @return
 */
bool Allocation::wasDispatched() const
{
  return wasAllocated() && state_ != states::ALLOCATED;
}

/**
 * @brief Allocation::wasAccepted
 * @return
 */
bool Allocation::wasAccepted() const
{
  return wasDispatched() && state_ != states::DISPATCHED;
}

/**
 * @brief Allocation::isExecuting
 * @return
 */
bool Allocation::isExecuting() const { return state_ == states::EXECUTING; }

/**
 * @brief Allocation::isFinished
 * @return
 */
bool Allocation::isFinished() const
{
  return state_ == states::SUCCEEDED || state_ == states::ABORTED ||
         state_ == states::CANCELLED;
}

/**
 * @brief Allocation::wasSucceeded
 * @return
 */
bool Allocation::wasSucceeded() const { return state_ == states::SUCCEEDED; }

/**
 * @brief Allocation::wasAborted
 * @return
 */
bool Allocation::wasAborted() const { return state_ == states::ABORTED; }

/**
 * @brief Allocation::wasCancelled
 * @return
 */
bool Allocation::wasCancelled() const { return state_ == states::CANCELLED; }

/**
 * @brief Allocation::wasEvaluated
 * @return
 */
bool Allocation::wasEvaluated() const { return isFinished(); }

/**
 * @brief Allocation::addRobots
 * @param robots
 */
void Allocation::addRobots(const std::vector<agents::Robot*>& robots)
{
  for (int i = 0; i < robots.size(); i++)
  {
    addRobot(robots[i]);
  }
}

/**
 * @brief Allocation::addRobot
 * @param robot
 */
void Allocation::addRobot(agents::Robot* robot)
{
  for (int i = 0; i < robots_.size(); i++)
  {
    if (*robot == *robots_[i])
    {
      return;
    }
  }
  robots_.push_back(robot);
}

/**
 * @brief Allocation::removeRobot
 * @param robot
 */
void Allocation::removeRobot(const agents::Robot& robot)
{
  for (int i = 0; i < robots_.size(); i++)
  {
    if (robot == *robots_[i])
    {
      robots_.erase(robots_.begin() + i);
      return;
    }
  }
}

/**
 * @brief Allocation::isValid
 * @param state
 * @return
 */
bool Allocation::isValid(AllocationStateEnum state)
{
  return state_ != states::SUCCEEDED &&
         (state == states::ABORTED || state == states::CANCELLED ||
          state_ != states::ABORTED && state_ != states::CANCELLED &&
              (state_ == states::NOT_ALLOCATED && state == states::ALLOCATED ||
               state_ == states::ALLOCATED && state == states::DISPATCHED ||
               state_ == states::DISPATCHED && state == states::EXECUTING ||
               state_ == states::EXECUTING && state == states::SUCCEEDED));
}

/**
 * @brief Allocation::setState
 * @param state
 */
void Allocation::setState(AllocationStateEnum state)
{
  if (isValid(state))
  {
    state_ = state;
  }
}

/**
 * @brief Allocation::hasStateChanged
 * @param state
 * @return
 */
bool Allocation::hasStateChanged(AllocationStateEnum state) const
{
  state_ != state;
}

/**
 * @brief Allocation::setSatisfaction
 * @param satisfaction
 */
void Allocation::setSatisfaction(AllocationSatisfactionEnum satisfaction)
{
  satisfaction_ = satisfaction;
}

/**
 * @brief Allocation::setAllocationTimestamp
 * @param allocation_timestamp
 */
void Allocation::setAllocationTimestamp(const ros::Time& allocation_timestamp)
{
  if (!allocation_timestamp_.isValid())
  {
    allocation_timestamp_ = allocation_timestamp;
  }
}

/**
 * @brief Allocation::setDispatchTimestamp
 * @param dispatch_timestamp
 */
void Allocation::setDispatchTimestamp(const ros::Time& dispatch_timestamp)
{
  if (allocation_timestamp_.isValid() && !dispatch_timestamp_.isValid() &&
      allocation_timestamp_ < dispatch_timestamp)
  {
    dispatch_timestamp_ = dispatch_timestamp;
  }
}

/**
 * @brief Allocation::setStartTimestamp
 * @param start_timestamp
 */
void Allocation::setStartTimestamp(const ros::Time& start_timestamp)
{
  if (dispatch_timestamp_.isValid() && !start_timestamp_.isValid() &&
      dispatch_timestamp_ < start_timestamp)
  {
    start_timestamp_ = start_timestamp;
  }
}

/**
 * @brief Allocation::setEndTimestamp
 * @param end_timestamp
 */
void Allocation::setEndTimestamp(const ros::Time& end_timestamp)
{
  if (start_timestamp_.isValid() && !end_timestamp_.isValid() &&
      start_timestamp_ < end_timestamp)
  {
    end_timestamp_ = end_timestamp;
  }
}

/**
 * @brief Allocation::allocate
 * @param robots
 */
void Allocation::allocate(const std::vector<agents::Robot*>& robots)
{
  if (!robots_.empty())
  {
    return;
  }
  addRobots(robots);
  setState(states::ALLOCATED);
  if (hasStateChanged(states::ALLOCATED))
  {
    setAllocationTimestamp();
  }
}

/**
 * @brief Allocation::dispatch
 */
void Allocation::dispatch()
{
  setState(states::DISPATCHED);
  if (hasStateChanged(states::DISPATCHED))
  {
    setAllocationTimestamp();
  }
}

/**
 * @brief Allocation::start
 */
void Allocation::start()
{
  setState(states::EXECUTING);
  if (hasStateChanged(states::EXECUTING))
  {
    setStartTimestamp();
  }
}

/**
 * @brief Allocation::end
 */
void Allocation::end()
{
  setState(states::SUCCEEDED);
  if (hasStateChanged(states::SUCCEEDED))
  {
    setEndTimestamp();
  }
}

/**
 * @brief Allocation::abort
 */
void Allocation::abort()
{
  setState(states::ABORTED);
  if (hasStateChanged(states::ABORTED))
  {
    setEndTimestamp();
  }
}

/**
 * @brief Allocation::cancel
 */
void Allocation::cancel()
{
  setState(states::CANCELLED);
  if (hasStateChanged(states::CANCELLED))
  {
    setEndTimestamp();
  }
}

/**
 * @brief Allocation::finish
 * @param state
 * @return
 */
bool Allocation::finish(states::AllocationStateEnum state)
{
  if (!isValid(state))
  {
    return false;
  }
  switch (state)
  {
  case states::SUCCEEDED:
    end();
    break;
  case states::ABORTED:
    abort();
    break;
  case states::CANCELLED:
    cancel();
    break;
  default:
    return false;
  }
  return true;
}

/**
 * @brief Allocation::isInvolved
 * @param robot
 * @return
 */
bool Allocation::isInvolved(const agents::Robot& robot) const
{
  for (int i = 0; i < robots_.size(); i++)
  {
    if (robot == *robots_[i])
    {
      return true;
    }
  }
  return false;
}

/**
 * @brief Allocation::isInvolved
 * @param person
 * @return
 */
bool Allocation::isInvolved(const agents::Person& person) const
{
  return task_->isInvolved(person);
}

/**
 * @brief Allocation::to_msg
 * @return
 */
mas_msgs::Allocation Allocation::to_msg() const
{
  mas_msgs::Allocation allocation_msg;
  if (task_)
  {
    allocation_msg.task = task_->to_msg();
  }
  for (int i = 0; i < robots_.size(); i++)
  {
    if (robots_[i])
    {
      allocation_msg.robots.push_back(robots_[i]->to_msg());
    }
  }
  allocation_msg.state = AllocationStates::toCode(state_);
  allocation_msg.satisfaction = AllocationSatisfactions::toCode(satisfaction_);
  allocation_msg.allocation_timestamp = allocation_timestamp_;
  allocation_msg.dispatch_timestamp = dispatch_timestamp_;
  allocation_msg.start_timestamp = start_timestamp_;
  allocation_msg.end_timestamp = end_timestamp_;
  return allocation_msg;
}

/**
 * @brief Allocation::str
 * @return
 */
std::string Allocation::str() const
{
  std::stringstream robots_ss;
  if (!robots_.empty())
  {
    if (robots_[0])
    {
      robots_ss << "0 " << robots_[0]->str();
    }
  }
  for (int i = 1; i < robots_.size(); i++)
  {
    if (robots_[i])
    {
      robots_ss << ", " << i << " " << robots_[i]->str();
    }
  }
  return "allocation: {" + (task_ ? task_->str() : "") +
         (!robots_ss.str().empty() ? ", robots: {" + robots_ss.str() + "}"
                                   : "") +
         ", state: " + AllocationStates::str(state_) + ", satisfaction: " +
         AllocationSatisfactions::str(satisfaction_) +
         ", allocation timestamp: " +
         utilities::TimeManipulator::str(allocation_timestamp_) +
         ", dispatch timestamp: " +
         utilities::TimeManipulator::str(dispatch_timestamp_) +
         ", start timestamp: " +
         utilities::TimeManipulator::str(start_timestamp_) +
         ", end timestamp: " + utilities::TimeManipulator::str(end_timestamp_) +
         "}";
}

/**
 * @brief Allocation::c_str
 * @return
 */
const char* Allocation::c_str() const { return str().c_str(); }

/**
 * @brief Allocation::operator =
 * @param allocation
 */
void Allocation::operator=(const Allocation& allocation)
{
  task_ = allocation.task_;
  robots_ = allocation.robots_;
  state_ = allocation.state_;
  satisfaction_ = allocation.satisfaction_;
  allocation_timestamp_ = allocation.allocation_timestamp_;
  dispatch_timestamp_ = allocation.dispatch_timestamp_;
  start_timestamp_ = allocation.start_timestamp_;
  end_timestamp_ = allocation.end_timestamp_;
}

/**
 * @brief Allocation::operator ==
 * @param allocation
 * @return
 */
bool Allocation::operator==(const Allocation& allocation) const
{
  return *task_ == *allocation.task_;
}

/**
 * @brief Allocation::operator !=
 * @param allocation
 * @return
 */
bool Allocation::operator!=(const Allocation& allocation) const
{
  return !operator==(allocation);
}

/**
 * @brief Allocation::compareTo
 * @param allocation
 * @return
 */
int Allocation::compareTo(const Allocation& allocation) const
{
  return task_->compareTo(*allocation.task_);
}
}
}
