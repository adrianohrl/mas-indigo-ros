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
#include "unifei/expertinos/mrta_vc/tasks/AllocationStates.h"
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
					Allocation(Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots, AllocationStateEnum state, TaskSatisfactionEnum satisfaction, ros::Time allocation_timestamp, ros::Time dispatch_timestamp, ros::Time start_timestamp, ros::Time end_timestamp);
					Allocation(Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots = std::vector<unifei::expertinos::mrta_vc::agents::Robot>());
					Allocation(const ::mrta_vc::Allocation::ConstPtr& allocation_msg);
					Allocation(::mrta_vc::Allocation allocation_msg);		
					~Allocation();

					Task getTask();
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> getRobots();
					AllocationStateEnum getState();
					TaskSatisfactionEnum getSatisfaction();
					ros::Time getAllocationTimestamp();
					ros::Time getDispatchTimestamp();
					ros::Time getStartTimestamp();
					ros::Time getEndTimestamp();
					bool wasAllocated();
					bool wasDispatched();
					bool wasAccepted();
					bool isExecuting();
					bool isFinished();
					bool wasSucceeded();
					bool wasAborted();
					bool wasCancelled();
					bool wasEvaluated();
					void setSatisfaction(TaskSatisfactionEnum satisfaction);
					void allocate(std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots);
					void dispatch();
					void start();
					void end();
					void abort();
					void cancel();
					bool finish(AllocationStateEnum state);
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
					AllocationStateEnum state_;
					TaskSatisfactionEnum satisfaction_;
					ros::Time allocation_timestamp_;
					ros::Time dispatch_timestamp_;
					ros::Time start_timestamp_;
					ros::Time end_timestamp_;

					void addRobots(std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots);
					void addRobot(unifei::expertinos::mrta_vc::agents::Robot robot);
					void removeRobot(unifei::expertinos::mrta_vc::agents::Robot robot);
					bool isValid(AllocationStateEnum state);
					void setState(AllocationStateEnum state);
					bool hasStateChanged(AllocationStateEnum state);
					void setAllocationTimestamp(ros::Time allocation_timestamp = ros::Time::now());
					void setDispatchTimestamp(ros::Time dispatch_timestamp = ros::Time::now());
					void setStartTimestamp(ros::Time start_timestamp = ros::Time::now());
					void setEndTimestamp(ros::Time end_timestamp = ros::Time::now());
					
				};
			}
		}
	}
}		

#endif /* ALLOCATION_H_ */
