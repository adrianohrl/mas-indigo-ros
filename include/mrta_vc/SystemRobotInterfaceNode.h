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
#include "mrta_vc/FinishAllocation.h"
#include "unifei/expertinos/mrta_vc/agents/Robot.h"
#include "unifei/expertinos/mrta_vc/tasks/Allocation.h"

#define ALLOCATION_INTERVAL_DURATION 2

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
		ros::Timer allocation_timer_;
		ros::Publisher beacon_pub_;
		ros::Publisher allocation_pub_;
		ros::Subscriber allocation_sub_;
		ros::Subscriber allocation_cancellation_sub_;
		ros::Subscriber allocation_abortion_sub_;
		ros::ServiceServer finish_allocation_srv_;
    bool setted_up_;
		unifei::expertinos::mrta_vc::tasks::Allocation allocation_;

		void beaconTimerCallback(const ros::TimerEvent& event);
		void taskEndTimerCallback(const ros::TimerEvent& event); // para testes
		void allocationTimerCallback(const ros::TimerEvent& event);
		void allocationsCallback(const mrta_vc::Allocation::ConstPtr& allocation_msg);
		void allocationCancellationsCallback(const mrta_vc::Task::ConstPtr& allocation_msg);
		void allocationAbortionsCallback(const mrta_vc::Task::ConstPtr& allocation_msg);
		bool finishAllocationCallback(mrta_vc::FinishAllocation::Request& request, mrta_vc::FinishAllocation::Response& response);
		void setUp();
		bool isAvailable();
		bool isBusy();

	};

}

#endif /* SYSTEM_ROBOT_INTERFACE_NODE_H_ */
