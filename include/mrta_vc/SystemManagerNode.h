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

#define TASK_INTERVAL_DURATION 2.0

namespace mrta_vc 
{

  class SystemManagerNode : public unifei::expertinos::mrta_vc::system::AllocationManager
	{

	public:

		/** Construtors */
		SystemManagerNode(ros::NodeHandle nh);
		/** Destrutor */
		~SystemManagerNode();

		/** métodos publicos relacionados ao gerenciamento do nó */
		void spin();

	private:
	
		/** atributos privados relacionados ao nó */
    ros::NodeHandle nh_;
    ros::Subscriber robots_sub_;
    ros::Subscriber tasks_sub_;
    ros::Subscriber users_sub_;
    ros::Timer robots_timer_;
    ros::Timer tasks_timer_;
    ros::Timer users_timer_;

    void robotsCallback(const mrta_vc::Agent::ConstPtr& robot_msg);
    void tasksCallback(const mrta_vc::Task::ConstPtr& task_msg);
    void usersCallback(const mrta_vc::Agent::ConstPtr& user_msg);
    void robotsTimerCallback(const ros::TimerEvent& event);
    void tasksTimerCallback(const ros::TimerEvent& event);
    void usersTimerCallback(const ros::TimerEvent& event);

	};

}

#endif /* SYSTEM_MANAGER_NODE_H_ */
