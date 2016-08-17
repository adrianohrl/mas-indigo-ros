/**
 *  AllocationManager.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 11/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/system/AllocationManager.h"

/**
 *
 */
unifei::expertinos::mrta_vc::system::AllocationManager::AllocationManager() 
{	
}

/**
 *
 */
unifei::expertinos::mrta_vc::system::AllocationManager::~AllocationManager() 
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::TaskPriorityQueue unifei::expertinos::mrta_vc::system::AllocationManager::getUnallocatedTasks()
{
	return unallocated_tasks_;
}

/**
 *
 */
std::list<unifei::expertinos::mrta_vc::tasks::Task> unifei::expertinos::mrta_vc::system::AllocationManager::getAllocatedTasks()
{
	return allocated_tasks_;
}

/**
 *
 */
std::list<unifei::expertinos::mrta_vc::tasks::Task> unifei::expertinos::mrta_vc::system::AllocationManager::getRequestedTasks()
{
	//std::list<unifei::expertinos::mrta_vc::tasks::Task> requested_tasks;
	/*for (int i = 0; i < unallocated_tasks_.size(); i++)
		{
				requested_tasks.push_back(unallocated_tasks_.at(i));
		}
		for (int i = 0; i < allocated_tasks_.size(); i++)
		{
				requested_tasks.push_back(allocated_tasks_.at(i));
	}*/
	return allocated_tasks_;
}

/**
 *
 */
std::list<unifei::expertinos::mrta_vc::tasks::Allocation> unifei::expertinos::mrta_vc::system::AllocationManager::getAllocations()
{
	return allocations_;
}

/**
 *
 */
std::list<unifei::expertinos::mrta_vc::agents::Robot> unifei::expertinos::mrta_vc::system::AllocationManager::getAvailableRobots()
{
	return available_robots_;
}

/**
 *
 */
std::list<unifei::expertinos::mrta_vc::agents::Robot> unifei::expertinos::mrta_vc::system::AllocationManager::getBusyRobots()
{
	return busy_robots_;
}

/**
 *
 */
std::list<unifei::expertinos::mrta_vc::agents::Robot> unifei::expertinos::mrta_vc::system::AllocationManager::getLoggedRobots()
{
	std::list<unifei::expertinos::mrta_vc::agents::Robot> logged_robots;
	/*for (int i = 0; i < available_robots_.size(); i++)
		{
				logged_robots.push_back(available_robots_.at(i));
		}
		for (int i = 0; i < busy_robots_.size(); i++)
		{
				logged_robots.push_back(busy_robots_.at(i));
	}*/
	return logged_robots;
}

/**
 *
 */
std::list<unifei::expertinos::mrta_vc::agents::User> unifei::expertinos::mrta_vc::system::AllocationManager::getLoggedUsers()
{
	return logged_users_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::system::AllocationManager::areThereAnyAvailableRobots()
{
	return !available_robots_.empty();
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::system::AllocationManager::areThereAnyUnallocatedTasks()
{
	return !available_robots_.empty();
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::system::AllocationManager::isAvailable(unifei::expertinos::mrta_vc::agents::Robot robot)
{
	std::list<unifei::expertinos::mrta_vc::agents::Robot>::iterator robot_it = available_robots_.begin();
	while (robot_it != available_robots_.end())
	{
		if (robot == *robot_it)
		{
			return true;
		}
	}
	return false;
}

/**
 * This function adds the input allocation
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::add(unifei::expertinos::mrta_vc::tasks::Allocation allocation)
{
	std::list<unifei::expertinos::mrta_vc::tasks::Allocation>::iterator allocation_it = allocations_.begin();
	while (allocation_it != allocations_.end())
	{
		if (allocation == *allocation_it)
		{
			return;
		}
	}
	allocations_.push_back(allocation);
}

/**
 * This function adds the input task to the unallocated tasks group according to prioirity queue policy (no
 * duplicated are added to this group)
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::add(unifei::expertinos::mrta_vc::tasks::Task task)
{
	unallocated_tasks_.push(task);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::add(unifei::expertinos::mrta_vc::agents::Robot robot) 
{
	std::list<unifei::expertinos::mrta_vc::agents::Robot>::iterator robot_it = busy_robots_.begin();
	while (robot_it != busy_robots_.end())
	{
		if(robot == *robot_it)
		{
			robot_it->setLastBeaconTimestamp(robot.getLastBeaconTimestamp());
			robot_it->setLocation(robot.getLocation());
			return;
		}
		++robot_it;
	}
	robot_it = available_robots_.begin();
	while (robot_it != available_robots_.end())
	{
		if(robot == *robot_it)
		{
			robot_it->setLastBeaconTimestamp(robot.getLastBeaconTimestamp());
			robot_it->setLocation(robot.getLocation());
			return;
		}
		++robot_it;
	}
	available_robots_.push_back(robot);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::add(unifei::expertinos::mrta_vc::agents::User user)
{
	std::list<unifei::expertinos::mrta_vc::agents::User>::iterator user_it = logged_users_.begin();
	while (user_it != logged_users_.end())
	{
		if (user == *user_it)
		{
			user_it->setLastBeaconTimestamp(user.getLastBeaconTimestamp());
			user_it->setLocation(user.getLocation());
			return;
		}
		++user_it;
	}
	logged_users_.push_back(user);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::remove(unifei::expertinos::mrta_vc::tasks::Task task) 
{
	unallocated_tasks_.push(task);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::remove(unifei::expertinos::mrta_vc::agents::Robot robot) 
{
	available_robots_.remove(robot);
	busy_robots_.remove(robot);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::remove(unifei::expertinos::mrta_vc::agents::User user)
{
	logged_users_.remove(user);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedRobots() 
{
	available_robots_.remove_if(unifei::expertinos::mrta_vc::agents::Robot::isNotLoggedAnyMore);
	busy_robots_.remove_if(unifei::expertinos::mrta_vc::agents::Robot::isNotLoggedAnyMore);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedUsers() 
{
	logged_users_.remove_if(unifei::expertinos::mrta_vc::agents::User::isNotLoggedAnyMore);
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::agents::Robot> unifei::expertinos::mrta_vc::system::AllocationManager::getBestTeam(unifei::expertinos::mrta_vc::tasks::Task task)
{
	std::vector<unifei::expertinos::mrta_vc::agents::Robot> best_team;
	if (available_robots_.empty())
	{
		return best_team;
	}
	double best_utility = 0.0;
	unifei::expertinos::mrta_vc::agents::Robot best_robot;
	std::list<unifei::expertinos::mrta_vc::agents::Robot>::iterator it = available_robots_.begin();
	while (it != available_robots_.end())
	{
		double utility = it->getUtility(task);
		if (utility > best_utility)
		{
			best_utility = utility;
			best_robot = *it;
		}
		++it;
	}
	// tem q implementar ainda pro caso de vários robôs realizando a mesma tarefa!!!
	if (best_utility != 0.0)
	{
		best_team.push_back(best_robot);
	}
	return best_team;
}

/**
 * This function allocates the input task the the input robot group.
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::allocate(unifei::expertinos::mrta_vc::tasks::Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots)
{
	transfer(task);
	transfer(robots);
	dispatch(unifei::expertinos::mrta_vc::tasks::Allocation(task, robots));
}

/**
 * This function dispatched the input allocation.
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::dispatch(unifei::expertinos::mrta_vc::tasks::Allocation allocation)
{
	if (!allocation.wasDispatched())
	{
		allocation.dispatch();
	}
	else if (!allocation.isExecuting())
	{
		add(allocation);
	}
}

/**
 * This method calls allocate method each moment it finds a team to do an unallocated task
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::updateUnallocatedTasks()
{
	std::vector<unifei::expertinos::mrta_vc::tasks::Task> unallocated_tasks;
	for (int i = 0; i < unallocated_tasks_.size(); i++)
	{
		unifei::expertinos::mrta_vc::tasks::Task task = unallocated_tasks_.top();
		std::vector<unifei::expertinos::mrta_vc::agents::Robot> best_team = getBestTeam(task);
		if (!best_team.empty())
		{
			allocate(task, best_team);
		}
		else
		{
			unallocated_tasks.push_back(task);
			unallocated_tasks_.pop();
		}
	}
	for (int i = 0; i < unallocated_tasks.size(); i++)
	{
		unallocated_tasks_.push(unallocated_tasks.at(i));
	}
}

/**
 * This method removes evaluated allocations from system
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::updateAllocations(unifei::expertinos::mrta_vc::tasks::Allocation allocation)
{
	std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots(allocation.getRobots());
	if (allocation.isFinished() && !robots.empty())
	{
		transfer(robots);
	}
	if (allocation.wasEvaluated())
	{
		allocations_.remove(allocation);
	}
}

/**
 * Through this method, the input task stops being an unallocated task and becomes an allocated one.
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::transfer(unifei::expertinos::mrta_vc::tasks::Task task)
{
	unallocated_tasks_.pop(); // criar uma priority queue que remove por task
	allocated_tasks_.push_back(task);
}

/**
 * The input robot is transferred from available to busy robots group if it is available. Otherwise, it is
 * transfered from busy to available robots group.
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::transfer(unifei::expertinos::mrta_vc::agents::Robot robot)
{
	ROS_WARN("[TRANSFERING] robot: %s", robot.getHostname().c_str());
	if (isAvailable(robot))
	{
		available_robots_.remove(robot);
		busy_robots_.push_back(robot);
	}
	else
	{
		busy_robots_.remove(robot);
		available_robots_.push_back(robot);
	}
}

/**
 * The input robots are transferred from a robot group to another according to the above method policy.
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::transfer(std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots)
{
	ROS_ERROR("[TRANSFERING] robots");
	for (int i = 0; i < robots.size(); i++)
	{
		unifei::expertinos::mrta_vc::agents::Robot robot = robots.at(i);
		robot.setLastBeaconTimestamp();
		transfer(robot);
	}
}
