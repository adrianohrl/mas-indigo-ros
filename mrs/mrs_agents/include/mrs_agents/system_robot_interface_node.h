/**
 *  This header file defines the SystemRobotInterfaceNode class, which is based
 *on the ROSNode helper class. It controls the system_robot_interface_node.
 *
 *  Version: 1.4.0
 *  Created on: 26/03/2016
 *  Modified on: 06/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _SYSTEM_ROBOT_INTERFACE_NODE_H_
#define _SYSTEM_ROBOT_INTERFACE_NODE_H_

#include <sstream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mas_actions/ExecuteAction.h>
#include <mas_srvs/FinishAllocation.h>
#include <mas/agents/robot.h>
#include <mas/tasks/allocation.h>
#include <utilities/ros_node.h>

#define ALLOCATION_INTERVAL_DURATION 5

namespace mrs_agents
{

class SystemRobotInterfaceNode : public utilities::ROSNode
{

public:
  SystemRobotInterfaceNode(ros::NodeHandle* nh);
  virtual ~SystemRobotInterfaceNode();

private:
  ros::Timer beacon_timer_;
  ros::Timer task_end_timer_; // para testes
  ros::Publisher beacon_pub_;
  ros::ServiceServer finish_allocation_srv_;
  actionlib::SimpleActionServer<mas_actions::ExecuteAction> execute_action_srv_;
  bool setted_up_;
  mas::agents::Robot* robot_;
  mas::tasks::Allocation* allocation_;

  void beaconTimerCallback(const ros::TimerEvent& event);
  void taskEndTimerCallback(const ros::TimerEvent& event); // para testes
  bool finishAllocationCallback(mas_srvs::FinishAllocation::Request& request,
                                mas_srvs::FinishAllocation::Response& response);
  void executeCallback(const mas_actions::ExecuteGoal::ConstPtr& goal);
  void executingTask();
  void publishExecuteFeedback();
  void publishExecuteResult(std::string message = "");
  void setUp();
  bool isAvailable();
  bool isBusy();
  virtual void controlLoop();
};
}

#endif /* _SYSTEM_ROBOT_INTERFACE_NODE_H_ */
