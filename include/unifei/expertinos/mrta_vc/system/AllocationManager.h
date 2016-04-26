/**
 *  AllocationManager.h
 *
 *  Version: 1.0.0.0
 *  Created on: 11/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ALLOCATION_MANAGER_H_
#define ALLOCATION_MANAGER_H_

#include "unifei/expertinos/mrta_vc/tasks/Allocation.h"

#define MAXIMUM_ROBOT_BEACON_ABSENCE_DURATION 3 * ROBOT_BEACON_INTERVAL_DURATION
#define MAXIMUM_USER_BEACON_ABSENCE_DURATION 3 * USER_BEACON_INTERVAL_DURATION

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace system
			{
				class AllocationManager 
				{

				protected:
					AllocationManager();	
					~AllocationManager();
					
					std::vector<unifei::expertinos::mrta_vc::tasks::Task> getUnallocatedTasks();
					std::vector<unifei::expertinos::mrta_vc::tasks::Task> getAllocatedTasks();
					std::vector<unifei::expertinos::mrta_vc::tasks::Task> getRequestedTasks();
					std::vector<unifei::expertinos::mrta_vc::tasks::Allocation> getAllocations();
					std::vector<unifei::expertinos::mrta_vc::agents::VoiceCommander> getLoggedUsers();
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> getAvailableRobots();
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> getBusyRobots();
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> getLoggedRobots();
					void addTask(unifei::expertinos::mrta_vc::tasks::Task task);
					void removeTask(unifei::expertinos::mrta_vc::tasks::Task task);
					void addRobot(unifei::expertinos::mrta_vc::agents::Robot robot);
					void removeRobot(unifei::expertinos::mrta_vc::agents::Robot robot);
					void addUser(unifei::expertinos::mrta_vc::agents::VoiceCommander user);
					void removeUser(unifei::expertinos::mrta_vc::agents::VoiceCommander user);
					void updateLoggedRobots();
					void updateLoggedUsers();
					//void allocate(unifei::expertinos::mrta_vc::tasks::Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots);

				private:
					std::vector<unifei::expertinos::mrta_vc::tasks::Task> unallocated_tasks_;
					std::vector<unifei::expertinos::mrta_vc::tasks::Task> allocated_tasks_;
					std::vector<unifei::expertinos::mrta_vc::tasks::Allocation> allocations_;
					std::vector<unifei::expertinos::mrta_vc::agents::VoiceCommander> logged_users_;
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> available_robots_;
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> busy_robots_;
					
					//int grade(unifei::expertinos::mrta_vc::agents::Robot robot);

				};
			}
		}
	}
}		
					
#endif /* ALLOCATION_MANAGER_H_ */
