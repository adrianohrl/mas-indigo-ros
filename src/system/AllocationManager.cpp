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
std::vector<unifei::expertinos::mrta_vc::tasks::Task> unifei::expertinos::mrta_vc::system::AllocationManager::getUnallocatedTasks() 
{
	return unallocated_tasks_;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::Task> unifei::expertinos::mrta_vc::system::AllocationManager::getAllocatedTasks() 
{
	return allocated_tasks_;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::Task> unifei::expertinos::mrta_vc::system::AllocationManager::getRequestedTasks() 
{
	std::vector<unifei::expertinos::mrta_vc::tasks::Task> requested_tasks;
	for (int i = 0; i < unallocated_tasks_.size(); i++)
	{
		requested_tasks.push_back(unallocated_tasks_.at(i));
	}
	for (int i = 0; i < allocated_tasks_.size(); i++)
	{
		requested_tasks.push_back(allocated_tasks_.at(i));
	}
	return requested_tasks;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::agents::Robot> unifei::expertinos::mrta_vc::system::AllocationManager::getAvailableRobots() 
{
	return available_robots_;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::agents::Robot> unifei::expertinos::mrta_vc::system::AllocationManager::getBusyRobots() 
{
	return busy_robots_;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::agents::Robot> unifei::expertinos::mrta_vc::system::AllocationManager::getLoggedRobots() 
{
	std::vector<unifei::expertinos::mrta_vc::agents::Robot> logged_robots;
	for (int i = 0; i < available_robots_.size(); i++)
	{
		logged_robots.push_back(available_robots_.at(i));
	}
	for (int i = 0; i < busy_robots_.size(); i++)
	{
		logged_robots.push_back(busy_robots_.at(i));
	}
	return logged_robots;
}

/**
 *
 */
std::list<unifei::expertinos::mrta_vc::agents::VoiceCommander> unifei::expertinos::mrta_vc::system::AllocationManager::getLoggedUsers() 
{
	return logged_users_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::add(unifei::expertinos::mrta_vc::tasks::Task task) 
{
	for (int i = 0; i < unallocated_tasks_.size(); i++)
	{
		if(task.equals(unallocated_tasks_.at(i)))
		{
			return;
		}
	}
	unallocated_tasks_.push_back(task);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::add(unifei::expertinos::mrta_vc::agents::Robot robot) 
{
	for (int i = 0; i < available_robots_.size(); i++)
	{
		if(robot.equals(available_robots_.at(i)))
		{
			available_robots_.at(i).setLastBeaconTimestamp(robot.getLastBeaconTimestamp());
			return;
		}
	}
	for (int i = 0; i < busy_robots_.size(); i++)
	{
		if(robot.equals(busy_robots_.at(i)))
		{
			busy_robots_.at(i).setLastBeaconTimestamp(robot.getLastBeaconTimestamp());
			return;
		}
	}
	available_robots_.push_back(robot);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::add(unifei::expertinos::mrta_vc::agents::VoiceCommander user) 
{
	std::list<unifei::expertinos::mrta_vc::agents::VoiceCommander>::iterator it = logged_users_.begin();
	while (it != logged_users_.end())
	{
		if(user.equals(*it))
		{
			(*it).setLastBeaconTimestamp(user.getLastBeaconTimestamp());
			return;
		}
		++it;
	}
	logged_users_.push_back(user);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::remove(unifei::expertinos::mrta_vc::tasks::Task task) 
{
	for (int i = 0; i < unallocated_tasks_.size(); i++)
	{
		if(task.equals(unallocated_tasks_.at(i)))
		{
			unallocated_tasks_.erase(unallocated_tasks_.begin() + i);
			return;
		}
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::remove(unifei::expertinos::mrta_vc::agents::Robot robot) 
{
	for (int i = 0; i < available_robots_.size(); i++)
	{
		if(robot.equals(available_robots_.at(i)))
		{
			available_robots_.erase(available_robots_.begin() + i);
			return;
		}
	}
	for (int i = 0; i < busy_robots_.size(); i++)
	{
		if(robot.equals(busy_robots_.at(i)))
		{
			busy_robots_.erase(busy_robots_.begin() + i);
			return;
		}
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::remove(unifei::expertinos::mrta_vc::agents::VoiceCommander user) 
{
	logged_users_.remove(user);
}

/**
 *	IMPLEMENTAR
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedRobots() 
{
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedUsers() 
{
	logged_users_.remove_if(isNotLogged);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::system::AllocationManager::isNotLogged(unifei::expertinos::mrta_vc::agents::VoiceCommander user)
{
	return !user.isLogged();
}
