/**
 *  TaskAllocator.h
 *
 *  Version: 1.2.2
 *  Created on: 11/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_TASK_ALLOCATOR_H_
#define SYSTEM_TASK_ALLOCATOR_H_

#include <list>
#include "unifei/expertinos/mrta_vc/tasks/Allocation.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace system
      {
				class TaskAllocator 
        {

				protected:
					TaskAllocator();	
					~TaskAllocator();
					
          unifei::expertinos::mrta_vc::tasks::TaskPriorityQueue getUnallocatedTasks();
          std::list<unifei::expertinos::mrta_vc::tasks::Task> getAllocatedTasks();
          std::list<unifei::expertinos::mrta_vc::tasks::Task> getRequestedTasks();
					std::list<unifei::expertinos::mrta_vc::tasks::Allocation> getAllocations();
          std::list<unifei::expertinos::mrta_vc::agents::Robot> getAvailableRobots();
          std::list<unifei::expertinos::mrta_vc::agents::Robot> getBusyRobots();
          std::list<unifei::expertinos::mrta_vc::agents::Robot> getLoggedRobots();
					std::list<unifei::expertinos::mrta_vc::agents::User> getLoggedUsers();
					void add(unifei::expertinos::mrta_vc::tasks::Allocation allocation);
					void add(unifei::expertinos::mrta_vc::tasks::Task task);
          void add(unifei::expertinos::mrta_vc::agents::Robot robot);
          void add(unifei::expertinos::mrta_vc::agents::User user);
          void remove(unifei::expertinos::mrta_vc::tasks::Task task);
          void remove(unifei::expertinos::mrta_vc::agents::Robot robot);
          void remove(unifei::expertinos::mrta_vc::agents::User user);
          void updateLoggedRobots();
          void updateLoggedUsers();
          void updateUnallocatedTasks();
					void updateAllocations(unifei::expertinos::mrta_vc::tasks::Allocation allocation);
					bool areThereAnyAvailableRobots();
					bool areThereAnyUnallocatedTasks();
					virtual void dispatch(unifei::expertinos::mrta_vc::tasks::Allocation allocation);

				private:
          unifei::expertinos::mrta_vc::tasks::TaskPriorityQueue unallocated_tasks_;
          std::list<unifei::expertinos::mrta_vc::tasks::Task> allocated_tasks_;
					std::list<unifei::expertinos::mrta_vc::tasks::Allocation> allocations_;
          std::list<unifei::expertinos::mrta_vc::agents::Robot> available_robots_;
          std::list<unifei::expertinos::mrta_vc::agents::Robot> busy_robots_;
					std::list<unifei::expertinos::mrta_vc::agents::User> logged_users_;

          void allocate(unifei::expertinos::mrta_vc::tasks::Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots);
          void transfer(unifei::expertinos::mrta_vc::tasks::Task task);
          void transfer(unifei::expertinos::mrta_vc::agents::Robot robot);
          void transfer(std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots);
          std::vector<unifei::expertinos::mrta_vc::agents::Robot> getBestTeam(unifei::expertinos::mrta_vc::tasks::Task task);
					bool isAvailable(unifei::expertinos::mrta_vc::agents::Robot robot);

				};
			}
		}
	}
}		
					
#endif /* SYSTEM_TASK_ALLOCATOR_H_ */
