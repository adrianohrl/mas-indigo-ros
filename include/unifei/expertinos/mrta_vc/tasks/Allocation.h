/**
 *  Allocation.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ALLOCATION_H_
#define ALLOCATION_H_

#include "mrta_vc/Allocation.h"
#include "unifei/expertinos/mrta_vc/agents/Robot.h"
#include "unifei/expertinos/mrta_vc/tasks/Task.h"
#include "unifei/expertinos/mrta_vc/tasks/TaskStates.h"
#include "unifei/expertinos/mrta_vc/tasks/TaskSatisfactions.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace tasks
			{
				class Allocation 
				{
				public:
					Allocation();
					Allocation(Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots = std::vector<unifei::expertinos::mrta_vc::agents::Robot>(), TaskStateEnum state = TaskStates::getDefault(), TaskSatisfactionEnum satisfaction = TaskSatisfactions::getDefault());
					Allocation(Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots, TaskStateEnum state, TaskSatisfactionEnum satisfaction, ros::Time allocation_timestamp, ros::Time start_timestamp, ros::Time end_timestamp);
					Allocation(const ::mrta_vc::Allocation::ConstPtr& allocation_msg);
					Allocation(::mrta_vc::Allocation allocation_msg);		
					~Allocation();

					Task getTask();
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> getRobots();
					TaskStateEnum getState();
					TaskSatisfactionEnum getSatisfaction();
					ros::Time getAllocationTimestamp();
					ros::Time getStartTimestamp();
					ros::Time getEndTimestamp();
					bool wasEvaluated();
					bool wasAllocated();
					bool wasAccepted();
					bool isExecuting();
					bool isFinished();
					bool wasSucceeded();
					bool wasAborted();
					bool wasCancelled();
					void setSatisfaction(TaskSatisfactionEnum satisfaction);
					void allocate(std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots);
					void start();
					void end();
					void abort();
					void fail();
					void cancel();
					bool finish(TaskStateEnum state);
					bool isInvolved(unifei::expertinos::mrta_vc::agents::Robot robot);
					bool isInvolved(unifei::expertinos::mrta_vc::agents::Person person);
					::mrta_vc::Allocation toMsg();
					std::string toString();
					int compareTo(Allocation allocation);
					bool equals(const Allocation& allocation);
					bool operator==(const Allocation& allocation);
					bool operator!=(const Allocation& allocation);
					void operator=(const Allocation& allocation);

				private:
					Task task_;
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots_;
					TaskStateEnum state_;
					TaskSatisfactionEnum satisfaction_;
					ros::Time allocation_timestamp_;
					ros::Time start_timestamp_;
					ros::Time end_timestamp_;

					void addRobots(std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots);
					void addRobot(unifei::expertinos::mrta_vc::agents::Robot robot);
					void removeRobot(unifei::expertinos::mrta_vc::agents::Robot robot);
					bool isValid(TaskStateEnum state);
					void setState(TaskStateEnum state);
					bool hasStateChanged(TaskStateEnum state);
					void setAllocationTimestamp(ros::Time allocation_timestamp = ros::Time::now());
					void setStartTimestamp(ros::Time start_timestamp = ros::Time::now());
					void setEndTimestamp(ros::Time end_timestamp = ros::Time::now());
					
				};
			}
		}
	}
}		

#endif /* ALLOCATION_H_ */
