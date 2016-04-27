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
#include <list>

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
				
				public:
					static bool isNotLogged(unifei::expertinos::mrta_vc::agents::VoiceCommander user);

				protected:
					AllocationManager();	
					~AllocationManager();
					
					std::vector<unifei::expertinos::mrta_vc::tasks::Task> getUnallocatedTasks();
					std::vector<unifei::expertinos::mrta_vc::tasks::Task> getAllocatedTasks();
					std::vector<unifei::expertinos::mrta_vc::tasks::Task> getRequestedTasks();
					std::vector<unifei::expertinos::mrta_vc::tasks::Allocation> getAllocations();
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> getAvailableRobots();
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> getBusyRobots();
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> getLoggedRobots();
					std::list<unifei::expertinos::mrta_vc::agents::VoiceCommander> getLoggedUsers();
					void add(unifei::expertinos::mrta_vc::tasks::Task task);
					void add(unifei::expertinos::mrta_vc::agents::Robot robot);
					void add(unifei::expertinos::mrta_vc::agents::VoiceCommander user);
					void remove(unifei::expertinos::mrta_vc::tasks::Task task);
					void remove(unifei::expertinos::mrta_vc::agents::Robot robot);
					void remove(unifei::expertinos::mrta_vc::agents::VoiceCommander user);
					void updateLoggedRobots();
					void updateLoggedUsers();
					//void allocate(unifei::expertinos::mrta_vc::tasks::Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots);

				private:
					std::vector<unifei::expertinos::mrta_vc::tasks::Task> unallocated_tasks_;
					std::vector<unifei::expertinos::mrta_vc::tasks::Task> allocated_tasks_;
					std::vector<unifei::expertinos::mrta_vc::tasks::Allocation> allocations_;
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> available_robots_;
					std::vector<unifei::expertinos::mrta_vc::agents::Robot> busy_robots_;
					std::list<unifei::expertinos::mrta_vc::agents::VoiceCommander> logged_users_;
					
					//int grade(unifei::expertinos::mrta_vc::agents::Robot robot);

				};
			}
		}
	}
}		
					
#endif /* ALLOCATION_MANAGER_H_ */
