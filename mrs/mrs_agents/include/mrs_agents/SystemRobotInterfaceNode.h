/**
 *  SystemRobotInterfaceNode.h
 *
 *  Version: 1.2.4
 *  Created on: 26/03/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_ROBOT_INTERFACE_NODE_H_
#define SYSTEM_ROBOT_INTERFACE_NODE_H_

#include <sstream>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mas_actions/ExecuteAction.h>
#include <mas_srvs/FinishAllocation.h>
#include <mas/agents/Robot.h>
#include <mas/tasks/Allocation.h>

#define ALLOCATION_INTERVAL_DURATION 5

namespace mrs_agents 
{

  class SystemRobotInterfaceNode : public mas::agents::Robot
	{

	public:
		SystemRobotInterfaceNode(ros::NodeHandle nh);
		virtual ~SystemRobotInterfaceNode();

		void spin();

	private:
		ros::NodeHandle nh_;
		ros::Timer beacon_timer_;
		ros::Timer task_end_timer_; // para testes
		ros::Publisher beacon_pub_;
		ros::ServiceServer finish_allocation_srv_;
		actionlib::SimpleActionServer<mas_actions::ExecuteAction> execute_action_srv_;
    		bool setted_up_;
		mas::tasks::Allocation allocation_;

		void beaconTimerCallback(const ros::TimerEvent& event);
		void taskEndTimerCallback(const ros::TimerEvent& event); // para testes
		bool finishAllocationCallback(mas_srvs::FinishAllocation::Request& request, mas_srvs::FinishAllocation::Response& response);
		void executeCallback(const mas_actions::ExecuteGoal::ConstPtr& goal);
		void executingTask();
		void publishExecuteFeedback();
		void publishExecuteResult(std::string message = "");
		void setUp();
		bool isAvailable();
		bool isBusy();

	};

}

#endif /* SYSTEM_ROBOT_INTERFACE_NODE_H_ */
