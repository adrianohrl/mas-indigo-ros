/**
 *  This source file implements the SystemRobotInterfaceNode class, which is
 *based on the ROSNode helper class. It controls the
 *system_robot_interface_node.
 *
 *  Version: 1.4.0
 *  Created on: 26/03/2016
 *  Modified on: 06/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrs_agents/system_robot_interface_node.h"

using typename mas::agents::Robot;
// using typename mas::tasks::Allocation;
// using typename mas::tasks::AllocationStateEnum;
// using typename mas::tasks::AllocationStates;
using namespace mas::tasks; // because of allocation state enums

namespace mrs_agents
{

/**
 * @brief SystemRobotInterfaceNode::SystemRobotInterfaceNode
 * @param nh
 */
SystemRobotInterfaceNode::SystemRobotInterfaceNode(ros::NodeHandle* nh)
    : ROSNode(nh, 20), robot_(NULL), allocation_(NULL), setted_up_(false),
      execute_action_srv_(
          *nh, "/execute_task",
          boost::bind(&SystemRobotInterfaceNode::executeCallback, this, _1),
          false)
{
  beacon_timer_ =
      nh->createTimer(ros::Duration(ROBOT_BEACON_INTERVAL_DURATION),
                      &SystemRobotInterfaceNode::beaconTimerCallback, this);
  task_end_timer_ = nh->createTimer(
      ros::Duration(10), &SystemRobotInterfaceNode::taskEndTimerCallback,
      this); // para testes
  beacon_pub_ = nh->advertise<mas_msgs::Agent>("/robots", 1);
  setUp();
}

/**
 * @brief SystemRobotInterfaceNode::~SystemRobotInterfaceNode
 */
SystemRobotInterfaceNode::~SystemRobotInterfaceNode()
{
  beacon_timer_.stop();
  task_end_timer_.stop(); // para testes
  beacon_pub_.shutdown();
  execute_action_srv_.shutdown();
  if (robot_)
  {
    delete robot_;
    robot_ = NULL;
  }
  if (allocation_)
  {
    delete allocation_;
    allocation_ = NULL;
  }
}

/**
 * @brief SystemRobotInterfaceNode::beaconTimerCallback send a beacon signal of
 * this robot to the system manager.
 * @param event
 */
void SystemRobotInterfaceNode::beaconTimerCallback(const ros::TimerEvent& event)
{
  if (setted_up_ && robot_)
  {
    beacon_pub_.publish(robot_->to_msg());
  }
}

/**
 * @brief SystemRobotInterfaceNode::taskEndTimerCallback PARA TESTES
 * @param event
 */
void SystemRobotInterfaceNode::taskEndTimerCallback(
    const ros::TimerEvent& event)
{
  if (allocation_ && isBusy())
  {
    allocation_->end();
    ROS_INFO("Successfully ending %s task!",
             allocation_->getTask()->getName().c_str());
  }
}

/**
 * @brief SystemRobotInterfaceNode::finishAllocationCallback sets the current
 * allocation to a final state.
 * @param request
 * @param response
 * @return
 */
bool SystemRobotInterfaceNode::finishAllocationCallback(
    mas_srvs::FinishAllocation::Request& request,
    mas_srvs::FinishAllocation::Response& response)
{
  AllocationStateEnum state(AllocationStates::toEnumerated(request.state));
  response.valid = allocation_->finish(state);
  response.message = "Invalid state code!!!";
  if (response.valid)
  {
    response.message = "Finished!!!";
  }
  return response.valid;
}

/**
 * @brief SystemRobotInterfaceNode::executeCallback receives new allocations
 * from system manager.
 * @param goal
 */
void SystemRobotInterfaceNode::executeCallback(
    const mas_actions::ExecuteGoal::ConstPtr& goal)
{
  ros::Rate loop_rate(10);
  Allocation* allocation = new Allocation(goal->allocation);
  if (allocation_)
  {
    ROS_WARN("[GOAL CB] Got new allocation: %d, and this: %d",
             allocation->getTask()->getId(), allocation_->getTask()->getId());
  }
  if (!allocation->isInvolved(*robot_))
  {
    delete allocation;
    allocation = NULL;
    ROS_WARN("Not involved!!!");
    return;
  }
  allocation_ = allocation;
  allocation_->start();
  while (ROSNode::ok())
  {
    if (execute_action_srv_.isPreemptRequested())
    {
      if (execute_action_srv_.isNewGoalAvailable())
      {
        allocation =
            new Allocation(execute_action_srv_.acceptNewGoal()->allocation);
        if (*allocation == *allocation_)
        {
          delete allocation;
          allocation = NULL;
          ROS_WARN("You've already sent this allocation before!!!");
          continue;
        }

        if (allocation_)
        {
          ROS_WARN("[GOAL CB] Got new allocation while executing one: %d, and "
                   "this: %d",
                   allocation->getTask()->getId(),
                   allocation_->getTask()->getId());
        }
        if (!allocation->isInvolved(*robot_))
        {
          delete allocation;
          allocation = NULL;
          ROS_WARN("Not accepted!!!");
          //////////////////////////////////////////////////////////////////
          ROS_ERROR("What do I do here????"); //////////////////////////////
          //////////////////////////////////////////////////////////////////
          return;
        }
      }
      ROS_INFO("Canceling!!!");
      allocation_->cancel();
      publishExecuteResult();
      return;
    }
    executingTask();
    publishExecuteFeedback();
    if (allocation_->isFinished())
    {
      publishExecuteResult();
      ROS_INFO("Sending result!!!");
      return;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Aborting allocation: this node has been killed!!!");
  allocation_->abort();
  publishExecuteResult();
}

/**
 * @brief SystemRobotInterfaceNode::executingTask
 */
void SystemRobotInterfaceNode::executingTask() {}

/**
 * @brief SystemRobotInterfaceNode::publishExecuteFeedback prepares the execute
 * action feedback message to send to client.
 */
void SystemRobotInterfaceNode::publishExecuteFeedback()
{
  if (allocation_->isFinished())
  {
    return;
  }
  mas_actions::ExecuteFeedback feedback;
  feedback.allocation = allocation_->to_msg();
  execute_action_srv_.publishFeedback(feedback);
}

/**
 * @brief SystemRobotInterfaceNode::publishExecuteResult prepares the execute
 * action result message to send to client, and also prepares this server to
 * receive a new goal allocation
 * @param message
 */
void SystemRobotInterfaceNode::publishExecuteResult(std::string message)
{
  mas_actions::ExecuteResult result;
  result.allocation = allocation_->to_msg();
  result.message = message;
  switch (allocation_->getState())
  {
  case states::ABORTED:
    execute_action_srv_.setAborted(result, message);
    break;
  case states::CANCELLED:
    execute_action_srv_.setPreempted(result, message);
    break;
  case states::SUCCEEDED:
    execute_action_srv_.setSucceeded(result, message);
    break;
  default:
    ROS_ERROR("Unable to publish execute action result: NOT A FINAL STATE!!!");
    ROS_ERROR("state: %s", AllocationStates::c_str(allocation_->getState()));
    return;
  }
  allocation_ = NULL;
}

/**
 * @brief SystemRobotInterfaceNode::setUp sets this object to a robot according
 * to the giving YAML file when this application started.
 */
void SystemRobotInterfaceNode::setUp()
{
  ros::NodeHandle* nh = ROSNode::getNodeHandle();
  bool setted_up(false);
  std::string ns(ros::this_node::getName());

  std::string hostname;
  nh->param<std::string>(ns + "/hostname", hostname, "");
  setted_up = hostname != "";
  ROS_ERROR_COND(!setted_up, "Invalid robot hostname!!!");

  bool mobile, holonomic;
  nh->param<bool>(ns + "/mobile", mobile, false);
  nh->param<bool>(ns + "/holonomic", holonomic, false);

  double location_x, location_y, location_theta;
  nh->param<double>(ns + "/location/x", location_x, 0);
  nh->param<double>(ns + "/location/y", location_y, 0);
  nh->param<double>(ns + "/location/theta", location_theta, 0);

  if (robot_)
  {
    delete robot_;
    robot_ = NULL;
  }
  robot_ = new Robot(0, hostname, holonomic, mobile, location_x, location_y,
                     location_theta);

  ns.append("/skill");
  int counter(0);
  std::stringstream aux;
  aux << ns << counter;
  while (nh->hasParam(aux.str() + "/resource/name"))
  {
    std::string resource_name, resource_description, level_name;
    nh->param<std::string>(aux.str() + "/resource/name", resource_name, "");
    nh->param<std::string>(aux.str() + "/resource/description",
                           resource_description, "");
    nh->param<std::string>(aux.str() + "/level", level_name, "");
    Resource* resource = new Resource(0, resource_name, resource_description);
    SkillLevelEnum level(SkillLevels::toEnumerated(level_name));
    Skill* skill = new Skill(0, resource, level);
    robot_->addSkill(skill);
    aux.str("");
    aux << ns << ++counter;
  }
  if (setted_up)
  {
    ROS_INFO("This robot has been setted up!!!");
    ROS_INFO("Robot Info:");
    ROS_INFO("     hostname: %s", robot_->getHostname().c_str());
    ROS_INFO("     holonomic: %s", robot_->isHolonomic() ? "true" : "false");
    ROS_INFO("     mobile: %s", robot_->isMobile() ? "true" : "false");
    ROS_INFO("     location: (%f, %f, %f)", robot_->getLocation()->getX(),
             robot_->getLocation()->getY(), robot_->getLocation()->getTheta());
    std::vector<Skill*> skills(robot_->getSkills());
    for (int i(0); i < skills.size(); i++)
    {
      Skill* skill = skills[i];
      ROS_INFO("     skill %d:", i);
      ROS_INFO("          resource: %s",
               skill->getResource()->getName().c_str());
      ROS_INFO("          level: %s", SkillLevels::c_str(skill->getLevel()));
    }
    execute_action_srv_.start();
  }
  else
  {
    ROS_ERROR(
        "You must create and set a YAML file containing this robot info!!!");
  }
  setted_up_ = setted_up;
}

/**
 * @brief SystemRobotInterfaceNode::isAvailable checks if this robot is not
 * executing any task.
 * @return
 */
bool SystemRobotInterfaceNode::isAvailable()
{
  Task* task = NULL;
  if (allocation_)
  {
    task = allocation_->getTask();
  }
  return !task || task && task->getName().empty();
}

/**
 * @brief SystemRobotInterfaceNode::isBusy check if this robot is executing a
 * task.
 * @return
 */
bool SystemRobotInterfaceNode::isBusy() { return !isAvailable(); }

/**
 * @brief SystemRobotInterfaceNode::controlLoop
 */
void SystemRobotInterfaceNode::controlLoop() {}
}
