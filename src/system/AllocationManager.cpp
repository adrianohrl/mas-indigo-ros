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
std::vector<unifei::expertinos::mrta_vc::agents::VoiceCommander> unifei::expertinos::mrta_vc::system::AllocationManager::getLoggedUsers() 
{
	return logged_users_;
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
void unifei::expertinos::mrta_vc::system::AllocationManager::addTask(unifei::expertinos::mrta_vc::tasks::Task task) 
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
void unifei::expertinos::mrta_vc::system::AllocationManager::removeTask(unifei::expertinos::mrta_vc::tasks::Task task) 
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
void unifei::expertinos::mrta_vc::system::AllocationManager::addRobot(unifei::expertinos::mrta_vc::agents::Robot robot) 
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
void unifei::expertinos::mrta_vc::system::AllocationManager::removeRobot(unifei::expertinos::mrta_vc::agents::Robot robot) 
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
void unifei::expertinos::mrta_vc::system::AllocationManager::addUser(unifei::expertinos::mrta_vc::agents::VoiceCommander user) 
{
	for (int i = 0; i < logged_users_.size(); i++)
	{
		if(user.equals(logged_users_.at(i)))
		{
			logged_users_.at(i).setLastBeaconTimestamp(user.getLastBeaconTimestamp());
			return;
		}
	}
	logged_users_.push_back(user);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::removeUser(unifei::expertinos::mrta_vc::agents::VoiceCommander user) 
{
	for (int i = 0; i < logged_users_.size(); i++)
	{
		if(user.equals(logged_users_.at(i)))
		{
			logged_users_.erase(logged_users_.begin() + i);
			return;
		}
	}
}

/**
 *	IMPLEMENTAR
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedRobots() 
{
}

/**
 *	TESTAR
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedUsers() 
{
	for (int i = 0; i < logged_users_.size(); i++)
	{
		if((ros::Time::now() - logged_users_.at(i).getLastBeaconTimestamp()).toSec() > MAXIMUM_USER_BEACON_ABSENCE_DURATION)
		{
			logged_users_.erase(logged_users_.begin() + i);
			i--;
		}
	}
}
