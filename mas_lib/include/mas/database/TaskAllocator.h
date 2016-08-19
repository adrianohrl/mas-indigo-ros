/**
 *  TaskAllocator.h
 *
 *  Version: 1.2.4
 *  Created on: 11/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_TASK_ALLOCATOR_H_
#define SYSTEM_TASK_ALLOCATOR_H_

#include <list>
#include "mas/tasks/Allocation.h"

namespace mas 
{
	namespace database
	{
		class TaskAllocator 
		{

		protected:
			TaskAllocator();	
			~TaskAllocator();
			
			tasks::TaskPriorityQueue getUnallocatedTasks();
			std::list<tasks::Task> getAllocatedTasks();
			std::list<tasks::Task> getRequestedTasks();
			std::list<tasks::Allocation> getAllocations();
			std::list<agents::Robot> getAvailableRobots();
			std::list<agents::Robot> getBusyRobots();
			std::list<agents::Robot> getLoggedRobots();
			std::list<agents::User> getLoggedUsers();
			void add(tasks::Allocation allocation);
			void add(tasks::Task task);
			void add(agents::Robot robot);
			void add(agents::User user);
			void remove(tasks::Task task);
			void remove(agents::Robot robot);
			void remove(agents::User user);
			void updateLoggedRobots();
			void updateLoggedUsers();
			void updateUnallocatedTasks();
			void updateAllocations(tasks::Allocation allocation);
			bool areThereAnyAvailableRobots();
			bool areThereAnyUnallocatedTasks();
			virtual void dispatch(tasks::Allocation allocation);

		private:
			tasks::TaskPriorityQueue unallocated_tasks_;
			std::list<tasks::Task> allocated_tasks_;
			std::list<tasks::Allocation> allocations_;
			std::list<agents::Robot> available_robots_;
			std::list<agents::Robot> busy_robots_;
			std::list<agents::User> logged_users_;

			void allocate(tasks::Task task, std::vector<agents::Robot> robots);
			void transfer(tasks::Task task);
			void transfer(agents::Robot robot);
			void transfer(std::vector<agents::Robot> robots);
			std::vector<agents::Robot> getBestTeam(tasks::Task task);
			bool isAvailable(agents::Robot robot);

		};
	}
}		
					
#endif /* SYSTEM_TASK_ALLOCATOR_H_ */
