/**
 *  This header file defines the TaskAllocator class.
 *
 *  Version: 1.4.0
 *  Created on: 11/04/2016
 *  Modified on: 14/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _SYSTEM_TASK_ALLOCATOR_H_
#define _SYSTEM_TASK_ALLOCATOR_H_

#include <list>
#include "mas/tasks/allocation.h"

namespace mas
{
namespace database
{
class TaskAllocator
{
public:
  TaskAllocator();
  ~TaskAllocator();

  tasks::TaskPriorityQueue getUnallocatedTasks() const;
  std::list<tasks::Task*> getAllocatedTasks() const;
  std::list<tasks::Task*> getRequestedTasks() const;
  std::list<tasks::Allocation*> getAllocations() const;
  std::list<agents::Robot*> getAvailableRobots() const;
  std::list<agents::Robot*> getBusyRobots() const;
  std::list<agents::Robot*> getLoggedRobots() const;
  std::list<agents::User*> getLoggedUsers() const;
  void add(tasks::Allocation *allocation);
  void add(tasks::Task* task);
  void add(const agents::Robot &robot);
  void add(const agents::User &user);
  void remove(const tasks::Allocation& finished_allocation);
  void remove(const tasks::Task& task);
  void remove(const agents::Robot& robot);
  void remove(const agents::User& user);
  void updateLoggedRobots();
  void updateLoggedUsers();
  std::list<tasks::Allocation *> updateUnallocatedTasks();
  void updateAllocations(const tasks::Allocation& allocation);
  bool areThereAnyAvailableRobots();
  bool areThereAnyUnallocatedTasks();
  void dispatch(tasks::Allocation* allocation);

private:
  tasks::TaskPriorityQueue unallocated_tasks_;
  std::list<tasks::Task*> allocated_tasks_;
  std::list<tasks::Allocation*> allocations_;
  std::list<agents::Robot*> available_robots_;
  std::list<agents::Robot*> busy_robots_;
  std::list<agents::User*> logged_users_;
  tasks::Allocation* allocate(tasks::Task* task, std::vector<agents::Robot*> robots);
  void transfer(tasks::Task* task);
  void transfer(agents::Robot* robot);
  void transfer(std::vector<agents::Robot*> robots);
  std::vector<agents::Robot*> getBestTeam(const tasks::Task& task) const;
  bool isAvailable(const agents::Robot& robot) const;
};
}
}

#endif /* _SYSTEM_TASK_ALLOCATOR_H_ */
