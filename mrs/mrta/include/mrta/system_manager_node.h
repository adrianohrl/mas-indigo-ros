/**
 *  This header file defines the SystemManagerNode class, which is based on
 *the ROSNode helper class. It controls the system_manager_node.
 *
 *  Version: 1.4.0
 *  Created on: 26/03/2016
 *  Modified on: 04/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _SYSTEM_MANAGER_NODE_H_
#define _SYSTEM_MANAGER_NODE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <mas_msgs/ManagerState.h>
#include <mas_actions/ExecuteAction.h>
#include <mas/database/task_allocator.h>
#include <utilities/ros_node.h>

#define TASK_INTERVAL_DURATION 2.0

namespace mrta
{

class SystemManagerNode : public utilities::ROSNode
{

public:
  SystemManagerNode(ros::NodeHandle* nh);
  ~SystemManagerNode();

protected:
  void dispatch(mas::tasks::Allocation *allocation);

private:
  mas::database::TaskAllocator* allocator_;
  ros::Subscriber robots_sub_;
  ros::Subscriber tasks_sub_;
  ros::Subscriber users_sub_;
  ros::Publisher manager_state_pub_;
  ros::Timer robots_timer_;
  ros::Timer tasks_timer_;
  ros::Timer users_timer_;
  ros::Timer allocation_timer_;
  actionlib::SimpleActionClient<mas_actions::ExecuteAction> execute_action_cli_;

  void robotsCallback(const mas_msgs::Agent::ConstPtr& robot_msg);
  void tasksCallback(const mas_msgs::Task::ConstPtr& task_msg);
  void usersCallback(const mas_msgs::Agent::ConstPtr& user_msg);
  void robotsTimerCallback(const ros::TimerEvent& event);
  void tasksTimerCallback(const ros::TimerEvent& event);
  void usersTimerCallback(const ros::TimerEvent& event);
  void allocationTimerCallback(const ros::TimerEvent& event);
  void allocationActiveCallback();
  void allocationFeedbackCallback(
      const mas_actions::ExecuteFeedback::ConstPtr& feedback);
  void
  allocationResultCallback(const actionlib::SimpleClientGoalState& state,
                           const mas_actions::ExecuteResult::ConstPtr& result);
  virtual void controlLoop();
};
}

#endif /* _SYSTEM_MANAGER_NODE_H_ */
