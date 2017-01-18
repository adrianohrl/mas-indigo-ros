/**
 *  This header file defines the SystemUserInterfaceNode class, which is based
 *on the ROSNode class. It controls the system_user_interface_node.
 *
 *  Version: 1.4.0
 *  Created on: 26/03/2016
 *  Modified on: 16/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _SYSTEM_USER_INTERFACE_NODE_H_
#define _SYSTEM_USER_INTERFACE_NODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mas_srvs/GetComputer.h>
#include <mas_srvs/GetUser.h>
#include <mas_srvs/ValidatePassword.h>
#include <mas/agents/user.h>
#include <mas/tasks/allocation.h>
#include <utilities/ros_node.h>

namespace mas_agents
{

class SystemUserInterfaceNode : public utilities::ROSNode
{
public:
  SystemUserInterfaceNode(ros::NodeHandle* nh);
  virtual ~SystemUserInterfaceNode();

private:
  mas::agents::User* user_;
  ros::Timer beacon_timer_;
  ros::Publisher beacon_pub_;
  ros::Publisher allocation_pub_;
  ros::Subscriber message_sub_;
  ros::Subscriber allocation_sub_;
  ros::ServiceServer get_user_srv_;
  ros::ServiceClient get_computer_cli_;
  ros::ServiceClient validate_cli_;
  std::list<mas::tasks::Allocation*> allocations_;
  bool logged_;
  virtual void controlLoop();
  void beaconTimerCallback(const ros::TimerEvent& event);
  void messagesCallback(const std_msgs::String::ConstPtr& message_msg);
  void
  allocationsCallback(const mas_msgs::Allocation::ConstPtr& allocation_msg);
  bool getUser(mas_srvs::GetUser::Request& request,
               mas_srvs::GetUser::Response& response);
  void login();
  void login(std::string login_name, std::string password);
  void logout();
  void setComputerUp();
  bool hasAssignedAnyTask() const;
};
}

#endif /* _SYSTEM_USER_INTERFACE_NODE_H_ */
