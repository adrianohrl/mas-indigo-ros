/**
 *  This source file implements the SystemManagerNode class, which is based on
 *the ROSNode helper class. It controls the system_manager_node.
 *
 *  Version: 1.4.0
 *  Created on: 26/03/2016
 *  Modified on: 17/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta/system_manager_node.h"

using typename mas::agents::Robot;
using typename mas::agents::User;
using typename mas::database::TaskAllocator;
using typename mas::tasks::Allocation;
using typename mas::tasks::Skill;
using typename mas::tasks::SkillLevels;
using typename mas::tasks::Task;

namespace mrta
{

/**
 * @brief SystemManagerNode::SystemManagerNode
 * @param nh
 */
SystemManagerNode::SystemManagerNode(ros::NodeHandle* nh)
    : ROSNode(nh, 40), execute_action_cli_("/execute_task", true),
      allocator_(new TaskAllocator())
{
  robots_sub_ =
      nh->subscribe("/robots", 100, &SystemManagerNode::robotsCallback, this);
  tasks_sub_ =
      nh->subscribe("/tasks", 100, &SystemManagerNode::tasksCallback, this);
  users_sub_ =
      nh->subscribe("/users", 100, &SystemManagerNode::usersCallback, this);
  manager_state_pub_ =
      nh->advertise<mas_msgs::ManagerState>("/manager_state", 1);
  robots_timer_ =
      nh->createTimer(ros::Duration(.75 * ROBOT_BEACON_INTERVAL_DURATION),
                      &SystemManagerNode::robotsTimerCallback, this);
  tasks_timer_ = nh->createTimer(ros::Duration(TASK_INTERVAL_DURATION),
                                 &SystemManagerNode::tasksTimerCallback, this);
  users_timer_ =
      nh->createTimer(ros::Duration(.75 * USER_BEACON_INTERVAL_DURATION),
                      &SystemManagerNode::usersTimerCallback, this);
}

/**
 * @brief SystemManagerNode::~SystemManagerNode
 */
SystemManagerNode::~SystemManagerNode()
{
  execute_action_cli_.stopTrackingGoal();
  robots_timer_.stop();
  tasks_timer_.stop();
  users_timer_.stop();
  robots_sub_.shutdown();
  tasks_sub_.shutdown();
  users_sub_.shutdown();
  manager_state_pub_.shutdown();
  if (allocator_)
  {
    delete allocator_;
    allocator_ = NULL;
  }
}

/**
 * @brief SystemManagerNode::robotsCallback receives new robot beacon signals in
 * order to keep track of which robots are still logged in the system.
 * @param robot_msg
 */
void SystemManagerNode::robotsCallback(
    const mas_msgs::Agent::ConstPtr& robot_msg)
{
  allocator_->add(Robot(robot_msg));
}

/**
 * @brief SystemManagerNode::tasksCallback receives new valid tasks.
 * @param task_msg
 */
void SystemManagerNode::tasksCallback(const mas_msgs::Task::ConstPtr& task_msg)
{
  Task* task = new Task(task_msg);
  allocator_->add(task);

  /************** Showing new task info 17/08/2016*****/
  // task = allocator_->getUnallocatedTasks().top();
  ROS_WARN("[MANAGER] ------- NEW TASK -------");
  ROS_ERROR("[MANAGER] id: %d, name: %s", task->getId(),
            task->getName().c_str());
  ROS_INFO("[MANAGER] skills:");
  std::vector<Skill*> skills(task->getSkills());
  for (int i = 0; i < skills.size(); i++)
  {
    Skill* skill = skills[i];
    ROS_INFO("[MANAGER] %s %s", SkillLevels::str(skill->getLevel()).c_str(),
             skill->getResource()->getName().c_str());
  }
  ROS_ERROR("[MANAGER] deadline: %s",
            utilities::TimeManipulator::str(task->getDeadline()).c_str());
  /****************************************************/
}

/**
 * @brief SystemManagerNode::usersCallback receives new user beacon signals so
 * that system can keep track of who is still logged in.
 * @param user_msg
 */
void SystemManagerNode::usersCallback(const mas_msgs::Agent::ConstPtr& user_msg)
{
  allocator_->add(User(user_msg));
}

/**
 * @brief SystemManagerNode::robotsTimerCallback triggers the robots logon
 * verification.
 * @param event
 */
void SystemManagerNode::robotsTimerCallback(const ros::TimerEvent& event)
{
  allocator_->updateLoggedRobots();
}

/**
 * @brief SystemManagerNode::tasksTimerCallback triggers the (new) allocation
 * verification.
 * @param event
 */
void SystemManagerNode::tasksTimerCallback(const ros::TimerEvent& event)
{
  std::list<Allocation*> allocations(allocator_->updateUnallocatedTasks());
  std::list<Allocation*>::iterator it(allocations.begin());
  while (it != allocations.end())
  {
    dispatch(*it);
    it++;
  }
}

/**
 * @brief SystemManagerNode::usersTimerCallback triggers the users logon
 * verification.
 * @param event
 */
void SystemManagerNode::usersTimerCallback(const ros::TimerEvent& event)
{
  allocator_->updateLoggedUsers();
}

/**
 * @brief SystemManagerNode::dispatch
 * @param allocation
 */
void SystemManagerNode::dispatch(Allocation* allocation)
{
  ROS_ERROR_COND(allocator_->areThereAnyAvailableRobots() &&
                     !execute_action_cli_.isServerConnected(),
                 "There is no execute server connected!!!");
  if (!allocation->wasDispatched())
  {
    ROS_DEBUG("[MANAGER] Sending %s allocation!!!",
              allocation->getTask()->getName().c_str());
    allocation->dispatch();
    if (allocation->wasDispatched())
    {
      mas_actions::ExecuteGoal goal;
      goal.allocation = allocation->to_msg();
      execute_action_cli_.sendGoal(
          goal, boost::bind(&SystemManagerNode::allocationResultCallback, this,
                            _1, _2),
          boost::bind(&SystemManagerNode::allocationActiveCallback, this),
          boost::bind(&SystemManagerNode::allocationFeedbackCallback, this,
                      _1));
      allocator_->dispatch(allocation);
    }
  }
}

/**
 * @brief SystemManagerNode::controlLoop
 */
void SystemManagerNode::controlLoop()
{
  mas_msgs::ManagerState manager_state_msg;
  manager_state_msg.number_of_unallocated_tasks =
      allocator_->getUnallocatedTasks().size();
  manager_state_msg.number_of_allocated_tasks =
      allocator_->getAllocatedTasks().size();
  manager_state_msg.number_of_available_robots =
      allocator_->getAvailableRobots().size();
  manager_state_msg.number_of_busy_robots = allocator_->getBusyRobots().size();
  manager_state_msg.number_of_logged_users =
      allocator_->getLoggedUsers().size();
  manager_state_msg.number_of_allocations = allocator_->getAllocations().size();
  manager_state_pub_.publish(manager_state_msg);
}

/**
 * @brief SystemManagerNode::allocationActiveCallback
 */
void SystemManagerNode::allocationActiveCallback()
{
  ROS_INFO("Goal just went active!!!");
}

/**
 * @brief SystemManagerNode::allocationFeedbackCallback
 * @param feedback
 */
void SystemManagerNode::allocationFeedbackCallback(
    const mas_actions::ExecuteFeedback::ConstPtr& feedback)
{
  Allocation allocation(feedback->allocation);
  // ROS_INFO("%s allocation state: %s", allocation.getTask()->getName().c_str(),
  // AllocationStates::c_str(allocation.getState()));
  allocator_->updateAllocations(allocation);
}

/**
 * @brief SystemManagerNode::allocationResultCallback
 * @param state
 * @param result
 */
void SystemManagerNode::allocationResultCallback(
    const actionlib::SimpleClientGoalState& state,
    const mas_actions::ExecuteResult::ConstPtr& result)
{
  Allocation allocation(result->allocation);
  ROS_INFO("%s allocation finished in state [%s]",
           allocation.getTask()->getName().c_str(), state.toString().c_str());
  allocator_->updateAllocations(allocation);
}
}
