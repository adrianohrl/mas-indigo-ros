/**
 *  SystemManagerNode.h
 *
 *  Version: 1.2.
 *  Created on: 26/03/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_MANAGER_NODE_H_
#define SYSTEM_MANAGER_NODE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <mas_msgs/ManagerState.h>
#include <mas_actions/ExecuteAction.h>
#include <mas/database/TaskAllocator.h>

#define TASK_INTERVAL_DURATION 2.0

namespace mrta 
{

  class SystemManagerNode : public mas::database::TaskAllocator
	{

	public:
		SystemManagerNode(ros::NodeHandle nh);
		~SystemManagerNode();

		void spin();

	protected:
		virtual void dispatch(mas::tasks::Allocation allocation);

	private:
		ros::NodeHandle nh_;
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
		void allocationFeedbackCallback(const mas_actions::ExecuteFeedback::ConstPtr& feedback);
		void allocationResultCallback(const actionlib::SimpleClientGoalState& state, const mas_actions::ExecuteResult::ConstPtr& result);
		void managerStatePublish();

	};
}

#endif /* SYSTEM_MANAGER_NODE_H_ */
