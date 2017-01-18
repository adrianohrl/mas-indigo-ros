/**
 *  This source file implements the SystemUserInterfaceNode class, which is
 *based on the Node helper class. It controls the system_user_interface_node.
 *
 *  Version: 1.4.0
 *  Created on: 26/03/2016
 *  Modified on: 16/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas_agents/system_user_interface_node.h"

using typename mas::agents::Computer;
using typename mas::agents::HierarchyLevels;
using typename mas::agents::Person;
using typename mas::agents::User;
using typename mas::tasks::Allocation;

namespace mas_agents
{

/**
 * @brief SystemUserInterfaceNode::SystemUserInterfaceNode
 * @param nh
 */
SystemUserInterfaceNode::SystemUserInterfaceNode(ros::NodeHandle* nh)
    : ROSNode(nh, 20), user_(NULL), logged_(false)
{
  beacon_timer_ =
      nh->createTimer(ros::Duration(USER_BEACON_INTERVAL_DURATION),
                      &SystemUserInterfaceNode::beaconTimerCallback, this);
  beacon_pub_ = nh->advertise<mas_msgs::Agent>("/users", 1);
  allocation_pub_ = nh->advertise<mas_msgs::Agent>("/finished_allocations", 1);
  message_sub_ = nh->subscribe(
      "messages", 1, &SystemUserInterfaceNode::messagesCallback, this);
  allocation_sub_ =
      nh->subscribe("/running_allocations", 1,
                    &SystemUserInterfaceNode::allocationsCallback, this);
  get_user_srv_ =
      nh->advertiseService("get_user", &SystemUserInterfaceNode::getUser, this);
  get_computer_cli_ = nh->serviceClient<mas_srvs::GetComputer>("/get_computer");
  validate_cli_ =
      nh->serviceClient<mas_srvs::ValidatePassword>("/validate_password");
  try
  {
    login();
  }
  catch (const utilities::Exception& e)
  {
    ROS_ERROR("%s", e.what());
  }
}

/**
 * @brief SystemUserInterfaceNode::~SystemUserInterfaceNode
 */
SystemUserInterfaceNode::~SystemUserInterfaceNode()
{
  logout();
  beacon_timer_.stop();
  beacon_pub_.shutdown();
  allocation_pub_.shutdown();
  message_sub_.shutdown();
  allocation_sub_.shutdown();
  get_user_srv_.shutdown();
  get_computer_cli_.shutdown();
  validate_cli_.shutdown();
  if (user_)
  {
    delete user_;
    user_ = NULL;
  }
  std::list<Allocation*>::iterator it(allocations_.begin());
  while (it != allocations_.end())
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
    it++;
  }
}

/**
 * @brief SystemUserInterfaceNode::controlLoop
 */
void SystemUserInterfaceNode::controlLoop() {}

/**
 * @brief SystemUserInterfaceNode::beaconTimerCallback
 * @param event
 */
void SystemUserInterfaceNode::beaconTimerCallback(const ros::TimerEvent& event)
{
  if (logged_)
  {
    beacon_pub_.publish(user_->to_msg());
  }
}

/**
 * @brief SystemUserInterfaceNode::messagesCallback
 * @param message_msg
 */
void SystemUserInterfaceNode::messagesCallback(
    const std_msgs::String::ConstPtr& message_msg)
{
  // ROS_INFO("[MESSAGE]: %s", message_msg->data.c_str());
}

/**
 * @brief SystemUserInterfaceNode::allocationsCallback
 * @param allocation_msg
 */
void SystemUserInterfaceNode::allocationsCallback(
    const mas_msgs::Allocation::ConstPtr& allocation_msg)
{
  if (logged_ && !hasAssignedAnyTask())
  {
    return;
  }
  Allocation* allocation = new Allocation(allocation_msg);
  if (allocation->isInvolved(*user_))
  {
    ROS_INFO("I'm involved in: %s", allocation->c_str());
    allocations_.push_back(allocation);
    // allocation_.start();
    // allocation_pub_.publish(allocation_.toMsg());
  }
  else if (allocation)
  {
    delete allocation;
    allocation = NULL;
  }
}

/**
 * @brief SystemUserInterfaceNode::getUser
 * @param request is not relevant
 * @param response
 * @return
 */
bool SystemUserInterfaceNode::getUser(mas_srvs::GetUser::Request &request,
                                      mas_srvs::GetUser::Response& response)
{
  response.logged = logged_;
  if (logged_)
  {
    response.message = user_->getName() + " is logged.";
    response.user = user_->to_msg();
  }
  else
  {
    response.message = "There is no user logged.";
  }
}

/**
 * @brief SystemUserInterfaceNode::login
 */
void SystemUserInterfaceNode::login()
{
  ros::NodeHandle* nh = ROSNode::getNodeHandle();
  std::string ns(ros::this_node::getName());
  if (nh->hasParam(ns + "/login_name") && nh->hasParam(ns + "/password"))
  {
    std::string login_name;
    std::string password;
    nh->param<std::string>(ns + "/login_name", login_name, "");
    nh->param<std::string>(ns + "/password", password, "");
    login(login_name, password);
  }
}

/**
 * @brief SystemUserInterfaceNode::login
 * @param login_name
 * @param password
 */
void SystemUserInterfaceNode::login(std::string login_name,
                                    std::string password)
{
  mas_srvs::ValidatePassword validate_srv;
  validate_srv.request.login_name = login_name;
  validate_srv.request.password = password;
  if (!validate_cli_.exists())
  {
    ROSNode::shutdown("The System Database Interface node is not running.");
  }
  if (!validate_cli_.call(validate_srv) || !validate_srv.response.valid)
  {
    ROSNode::shutdown("Login failure: " + validate_srv.response.message);
  }
  User* user = new User(validate_srv.response.user);
  user_ = user;
  setComputerUp();

  ROS_DEBUG("Login succeeded: %s", validate_srv.response.message.c_str());
  ROS_DEBUG("%s", user_->c_str());

  logged_ = true;
  ROS_INFO("User Info:");
  ROS_INFO("     id: %d", user_->getId());
  ROS_INFO("     name: %s", user_->getName().c_str());
  ROS_INFO("     hierarchy_level: %s",
           HierarchyLevels::c_str(user_->getHierarchyLevel()));
  ROS_INFO("     login_name: %s", user_->getLoginName().c_str());
  ROS_INFO("     computer:");
  ROS_INFO("          id: %d", user_->getComputer()->getId());
  ROS_INFO("          hostname: %s",
           user_->getComputer()->getHostname().c_str());
  ROS_INFO("          mobile: %s",
           user_->getComputer()->isMobile() ? "true" : "false");
  ROS_INFO("          location: (%f, %f, %f)",
           user_->getComputer()->getLocation()->getX(),
           user_->getComputer()->getLocation()->getY(),
           user_->getComputer()->getLocation()->getTheta());
  ROS_INFO("     location: (%f, %f, %f)", user_->getLocation()->getX(),
           user_->getLocation()->getY(), user_->getLocation()->getTheta());
  ROS_INFO("User is logged in!!!");
}

/**
 * @brief SystemUserInterfaceNode::logout
 */
void SystemUserInterfaceNode::logout()
{
  ROS_DEBUG("%s has been logged out!!!", user_->getLoginName().c_str());
  logged_ = false;
}

/**
 * @brief SystemUserInterfaceNode::setComputerUp
 */
void SystemUserInterfaceNode::setComputerUp()
{
  ros::NodeHandle* nh = ROSNode::getNodeHandle();
  std::string hostname;
  nh->param<std::string>(ros::this_node::getName() + "/computer/hostname",
                         hostname, "");
  if (hostname.empty())
  {
    ROSNode::shutdown(
        "You must create and set a YAML file containing this machine info.");
  }
  if (!get_computer_cli_.exists())
  {
    ROS_ERROR("Unable to set user's computer up.");
    ROSNode::shutdown("The System Database Interface node is not running.");
    return;
  }
  mas_srvs::GetComputer srv;
  srv.request.hostname = hostname;
  if (!get_computer_cli_.call(srv) || !srv.response.valid)
  {
    ROSNode::shutdown("Computer setting up failure: " + srv.response.message);
  }
  Computer* old = user_->getComputer();
  if (old)
  {
    delete old;
    old = NULL;
  }
  user_->setComputer(new Computer(srv.response.computer));
  ROS_INFO("This computer has been setted up!!!");
  ROS_DEBUG("Computer setting up succeeded: %s", srv.response.message.c_str());
  ROS_DEBUG("%s", user_->getComputer()->c_str());
}

/**
 * @brief SystemUserInterfaceNode::hasAssignedAnyTask
 * @return
 */
bool SystemUserInterfaceNode::hasAssignedAnyTask() const
{
  return !allocations_.empty();
}
}
