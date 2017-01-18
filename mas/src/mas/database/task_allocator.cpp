/**
 *  This source file implements the TaskAllocator class.
 *
 *  Version: 1.4.0
 *  Created on: 11/04/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/database/task_allocator.h"

namespace mas
{
namespace database
{

/**
 * @brief TaskAllocator::TaskAllocator
 */
TaskAllocator::TaskAllocator() {}

/**
 * @brief TaskAllocator::~TaskAllocator
 */
TaskAllocator::~TaskAllocator()
{
  std::list<agents::Robot*>::iterator robot_it(available_robots_.begin());
  while (robot_it != available_robots_.end())
  {
    robot_it++;
  }
  robot_it = busy_robots_.begin();
  while (robot_it != busy_robots_.end())
  {
    robot_it++;
  }
  std::list<agents::User*>::iterator user_it(logged_users_.begin());
  while (user_it != logged_users_.end())
  {
    user_it++;
  }
  /*while (!unallocated_tasks_.empty())
  {
  ///
  }*/
  std::list<tasks::Task*>::iterator task_it(allocated_tasks_.begin());
  while (task_it != allocated_tasks_.end())
  {
    task_it++;
  }
  std::list<tasks::Allocation*>::iterator allocation_it(allocations_.begin());
  while (allocation_it != allocations_.end())
  {
    allocation_it++;
  }
}

/**
 * @brief TaskAllocator::getUnallocatedTasks
 * @return
 */
tasks::TaskPriorityQueue TaskAllocator::getUnallocatedTasks() const
{
  return unallocated_tasks_;
}

/**
 * @brief TaskAllocator::getAllocatedTasks
 * @return
 */
std::list<tasks::Task*> TaskAllocator::getAllocatedTasks() const
{
  return allocated_tasks_;
}

/**
 * @brief TaskAllocator::getRequestedTasks
 * @return
 */
std::list<tasks::Task*> TaskAllocator::getRequestedTasks() const
{
  /*std::list<tasks::Task*> requested_tasks(allocated_tasks_);
  // tenho q varrer d alguma forma eficiente
  return requested_tasks;*/
  return allocated_tasks_;
}

/**
 * @brief TaskAllocator::getAllocations
 * @return
 */
std::list<tasks::Allocation*> TaskAllocator::getAllocations() const
{
  return allocations_;
}

/**
 * @brief TaskAllocator::getAvailableRobots
 * @return
 */
std::list<agents::Robot*> TaskAllocator::getAvailableRobots() const
{
  return available_robots_;
}

/**
 * @brief TaskAllocator::getBusyRobots
 * @return
 */
std::list<agents::Robot*> TaskAllocator::getBusyRobots() const
{
  return busy_robots_;
}

/**
 * @brief TaskAllocator::getLoggedRobots
 * @return
 */
std::list<agents::Robot*> TaskAllocator::getLoggedRobots() const
{
  std::list<agents::Robot*> logged_robots(available_robots_);
  logged_robots.insert(logged_robots.end(), busy_robots_.begin(),
                       busy_robots_.end());
  return logged_robots;
}

/**
 * @brief TaskAllocator::getLoggedUsers
 * @return
 */
std::list<agents::User*> TaskAllocator::getLoggedUsers() const
{
  return logged_users_;
}

/**
 * @brief TaskAllocator::areThereAnyAvailableRobots
 * @return
 */
bool TaskAllocator::areThereAnyAvailableRobots()
{
  return !available_robots_.empty();
}

/**
 * @brief TaskAllocator::areThereAnyUnallocatedTasks
 * @return
 */
bool TaskAllocator::areThereAnyUnallocatedTasks()
{
  return !available_robots_.empty();
}

/**
 * @brief TaskAllocator::isAvailable
 * @param robot
 * @return
 */
bool TaskAllocator::isAvailable(const agents::Robot& robot) const
{
  std::list<agents::Robot*>::const_iterator robot_it =
      available_robots_.begin();
  while (robot_it != available_robots_.end())
  {
    if (**robot_it == robot)
    {
      return true;
    }
  }
  return false;
}

/**
 * @brief TaskAllocator::add adds the input allocation
 * @param allocation
 */
void TaskAllocator::add(tasks::Allocation* allocation)
{
  std::list<tasks::Allocation*>::iterator allocation_it = allocations_.begin();
  while (allocation_it != allocations_.end())
  {
    if (**allocation_it == *allocation)
    {
      return;
    }
  }
  allocations_.push_back(allocation);
}

/**
 * @brief TaskAllocator::add adds the input task to the unallocated tasks group
 * according to prioirity queue policy (no
 * duplicated are added to this group)
 * @param task
 */
void TaskAllocator::add(tasks::Task* task)
{
  // implementar e testar
  unallocated_tasks_.push(task);
}

/**
 * @brief TaskAllocator::add
 * @param robot
 */
void TaskAllocator::add(const agents::Robot& robot)
{
  std::list<agents::Robot*>::iterator robot_it(busy_robots_.begin());
  while (robot_it != busy_robots_.end())
  {
    agents::Robot* busy_robot = *robot_it;
    if (*busy_robot == robot)
    {
      busy_robot->setLastBeaconTimestamp(robot.getLastBeaconTimestamp());
      busy_robot->setLocation(*robot.getLocation());
      return;
    }
    ++robot_it;
  }
  robot_it = available_robots_.begin();
  while (robot_it != available_robots_.end())
  {
    agents::Robot* available_robot = *robot_it;
    if (*available_robot == robot)
    {
      available_robot->setLastBeaconTimestamp(robot.getLastBeaconTimestamp());
      available_robot->setLocation(*robot.getLocation());
      return;
    }
    ++robot_it;
  }
  available_robots_.push_back(new agents::Robot(robot));
}

/**
 * @brief TaskAllocator::add
 * @param user
 */
void TaskAllocator::add(const agents::User& user)
{
  std::list<agents::User*>::iterator user_it(logged_users_.begin());
  while (user_it != logged_users_.end())
  {
    agents::User* logged_user = *user_it;
    if (*logged_user == user)
    {
      logged_user->setLastBeaconTimestamp(user.getLastBeaconTimestamp());
      logged_user->setLocation(*user.getLocation());
      return;
    }
    ++user_it;
  }
  logged_users_.push_back(new agents::User(user));
}

/**
 * @brief TaskAllocator::remove erases the input allocation.
 * @param allocation
 */
void TaskAllocator::remove(const tasks::Allocation &finished_allocation)
{
  std::list<tasks::Allocation*>::iterator it(allocations_.begin());
  while (it != allocations_.end())
  {
    tasks::Allocation* allocation = *it;
    if (*allocation == finished_allocation)
    {
      delete allocation;
      allocation = NULL;
      allocations_.erase(it);
      return;
    }
    ++it;
  }
}

/**
 * @brief TaskAllocator::remove erases the input allocated task.
 * @param task
 */
void TaskAllocator::remove(const tasks::Task& task)
{
  std::list<tasks::Task*>::iterator it(allocated_tasks_.begin());
  while (it != allocated_tasks_.end())
  {
    tasks::Task* allocated_task = *it;
    if (*allocated_task == task)
    {
      delete allocated_task;
      allocated_task = NULL;
      allocated_tasks_.erase(it);
      return;
    }
    ++it;
  }
}

/**
 * @brief TaskAllocator::remove
 * @param robot
 */
void TaskAllocator::remove(const agents::Robot& robot)
{
  std::list<agents::Robot*>::iterator it(busy_robots_.begin());
  while (it != busy_robots_.end())
  {
    agents::Robot* busy_robot = *it;
    if (*busy_robot == robot)
    {
      delete busy_robot;
      busy_robot = NULL;
      busy_robots_.erase(it);
      return;
    }
    ++it;
  }
  it = available_robots_.begin();
  while (it != available_robots_.end())
  {
    agents::Robot* available_robot = *it;
    if (*available_robot == robot)
    {
      delete available_robot;
      available_robot = NULL;
      available_robots_.erase(it);
      return;
    }
    ++it;
  }
}

/**
 * @brief TaskAllocator::remove
 * @param user
 */
void TaskAllocator::remove(const agents::User& user)
{
  std::list<agents::User*>::iterator it(logged_users_.begin());
  while (it != logged_users_.end())
  {
    agents::User* logged_user = *it;
    if (*logged_user == user)
    {
      delete logged_user;
      logged_user = NULL;
      logged_users_.erase(it);
      return;
    }
    ++it;
  }
}

/**
 * @brief TaskAllocator::updateLoggedRobots
 */
void TaskAllocator::updateLoggedRobots()
{
  available_robots_.remove_if(agents::Robot::isNotLoggedAnyMore); // need to deallocate memory
  busy_robots_.remove_if(agents::Robot::isNotLoggedAnyMore); // need to deallocate memory
}

/**
 * @brief TaskAllocator::updateLoggedUsers
 */
void TaskAllocator::updateLoggedUsers()
{
  logged_users_.remove_if(agents::User::isNotLoggedAnyMore);  // need to deallocate memory
}

/**
 * @brief TaskAllocator::getBestTeam
 * @param task
 * @return
 */
std::vector<agents::Robot*>
TaskAllocator::getBestTeam(const tasks::Task& task) const
{
  std::vector<agents::Robot*> best_team;
  if (available_robots_.empty())
  {
    return best_team;
  }
  double best_utility(0.0);
  agents::Robot* best_robot = NULL;
  std::list<agents::Robot*>::const_iterator it(available_robots_.begin());
  while (it != available_robots_.end())
  {
    agents::Robot* robot = *it;
    double utility(robot->getUtility(task));
    if (utility > best_utility)
    {
      best_utility = utility;
      best_robot = robot;
    }
    ++it;
  }
  // tem q implementar ainda pro caso de vários robôs realizando a mesma
  // tarefa!!!
  if (best_robot)
  {
    best_team.push_back(best_robot);
  }
  return best_team;
}

/**
 * @brief TaskAllocator::allocate allocates the input task the the input robot
 * @param task
 * @param robots
 * @return
 */
tasks::Allocation* TaskAllocator::allocate(tasks::Task* task,
                                           std::vector<agents::Robot*> robots)
{
  transfer(task);
  transfer(robots);
  return new tasks::Allocation(task, robots);
}

/**
 * @brief TaskAllocator::dispatch dispatches the input allocation.
 * @param allocation
 */
void TaskAllocator::dispatch(tasks::Allocation* allocation)
{
  if (!allocation->wasDispatched())
  {
    allocation->dispatch();
  }
  else if (!allocation->isExecuting())
  {
    add(allocation);
  }
}

/**
 * @brief TaskAllocator::updateUnallocatedTasks calls allocate method each
 * moment it finds a team to do an unallocated task.
 * @return
 */
std::list<tasks::Allocation*> TaskAllocator::updateUnallocatedTasks()
{
  std::list<tasks::Allocation*> allocations;
  std::vector<tasks::Task*> unallocated_tasks;
  while (!unallocated_tasks_.empty())
  {
    tasks::Task* task = unallocated_tasks_.top();
    std::vector<agents::Robot*> best_team(getBestTeam(*task));
    if (!best_team.empty())
    {
      tasks::Allocation* allocation = allocate(task, best_team);
      allocations.push_back(allocation);
    }
    else
    {
      unallocated_tasks.push_back(task);
      unallocated_tasks_.pop();
    }
  }
  for (int i(0); i < unallocated_tasks.size(); i++)
  {
    unallocated_tasks_.push(unallocated_tasks[i]);
  }
  return allocations;
}

/**
 * @brief TaskAllocator::updateAllocations removes evaluated allocations from
 * system.
 * @param allocation
 */
void TaskAllocator::updateAllocations(const tasks::Allocation& allocation)
{
  std::vector<agents::Robot*> robots(allocation.getRobots());
  if (allocation.isFinished() && !robots.empty())
  {
    transfer(robots);
  }
  if (allocation.wasEvaluated())
  {
    remove(allocation);
  }
}

/**
 * @brief TaskAllocator::transfer through this method, the input task stops
 * being an unallocated task and becomes an allocated one.
 * @param task
 */
void TaskAllocator::transfer(tasks::Task* task)
{
  unallocated_tasks_.pop(); // criar uma priority queue que remove por task
  allocated_tasks_.push_back(task);
}

/**
 * @brief TaskAllocator::transfer the input robot is transferred from available
 * to busy robots group if it is available. Otherwise, it is
 * transfered from busy to available robots group.
 * @param robot
 */
void TaskAllocator::transfer(agents::Robot* robot)
{
  if (isAvailable(*robot))
  {
    available_robots_.remove_if(agents::RobotComparator(robot)); // need to deallocate memory
    busy_robots_.push_back(robot);
  }
  else
  {
    busy_robots_.remove_if(agents::RobotComparator(robot)); // need to deallocate memory
    available_robots_.push_back(robot);
  }
}

/**
 * @brief TaskAllocator::transfer the input robots are transferred from a robot
 * group to another according to the above method policy.
 * @param robots
 */
void TaskAllocator::transfer(std::vector<agents::Robot*> robots)
{
  for (int i(0); i < robots.size(); i++)
  {
    agents::Robot* robot = robots[i];
    robot->setLastBeaconTimestamp();
    transfer(robot);
  }
}
}
}
