/**
 *  SystemManagerNode.h
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_MANAGER_NODE_H_
#define SYSTEM_MANAGER_NODE_H_

#include <ros/ros.h>
#include "unifei/expertinos/mrta_vc/system/AllocationManager.h"
#include "mrta_vc/Allocation.h"
#include "mrta_vc/ManagerState.h"

#define TASK_INTERVAL_DURATION 2.0
#define ALLOCATION_INTERVAL_DURATION 1.0

namespace mrta_vc 
{

  class SystemManagerNode : public unifei::expertinos::mrta_vc::system::AllocationManager
	{

	public:
		SystemManagerNode(ros::NodeHandle nh);
		~SystemManagerNode();

		void spin();

	private:
    ros::NodeHandle nh_;
    ros::Subscriber robots_sub_;
    ros::Subscriber tasks_sub_;
    ros::Subscriber users_sub_;
    ros::Subscriber allocation_sub_;
    ros::Publisher allocation_pub_;
    ros::Publisher manager_state_pub_;
    ros::Timer robots_timer_;
    ros::Timer tasks_timer_;
    ros::Timer users_timer_;
    ros::Timer allocation_timer_;

    void robotsCallback(const mrta_vc::Agent::ConstPtr& robot_msg);
    void tasksCallback(const mrta_vc::Task::ConstPtr& task_msg);
    void usersCallback(const mrta_vc::Agent::ConstPtr& user_msg);
    void allocationsCallback(const mrta_vc::Allocation::ConstPtr& allocation_msg);
    void robotsTimerCallback(const ros::TimerEvent& event);
    void tasksTimerCallback(const ros::TimerEvent& event);
    void usersTimerCallback(const ros::TimerEvent& event);
    void allocationTimerCallback(const ros::TimerEvent& event);

    void managerStatePublish();

	};

}

#endif /* SYSTEM_MANAGER_NODE_H_ */
