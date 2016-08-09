/**
 *  SystemRobotInterfaceNode.h
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_ROBOT_INTERFACE_NODE_H_
#define SYSTEM_ROBOT_INTERFACE_NODE_H_

#include <sstream>
#include <ros/ros.h>
#include <unifei/expertinos/mrta_vc/agents/Robot.h> /* libmas */
#include <unifei/expertinos/mrta_vc/tasks/Allocation.h> /* libmas */
#include <actionlib/server/simple_action_server.h>
#include "mrta_vc/ExecuteAction.h"
#include "mrta_vc/FinishAllocation.h"

#define ALLOCATION_INTERVAL_DURATION 5

namespace mrta_vc 
{

  class SystemRobotInterfaceNode : public unifei::expertinos::mrta_vc::agents::Robot
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
		actionlib::SimpleActionServer<mrta_vc::ExecuteAction> execute_action_srv_;
    bool setted_up_;
		unifei::expertinos::mrta_vc::tasks::Allocation allocation_;

		void beaconTimerCallback(const ros::TimerEvent& event);
		void taskEndTimerCallback(const ros::TimerEvent& event); // para testes
		bool finishAllocationCallback(mrta_vc::FinishAllocation::Request& request, mrta_vc::FinishAllocation::Response& response);
		void executeCallback(const mrta_vc::ExecuteGoal::ConstPtr& goal);
		void executingTask();
		void publishExecuteFeedback();
		void publishExecuteResult(std::string message = "");
		void setUp();
		bool isAvailable();
		bool isBusy();

	};

}

#endif /* SYSTEM_ROBOT_INTERFACE_NODE_H_ */
