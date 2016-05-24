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
void unifei::expertinos::mrta_vc::system::AllocationManager::add(unifei::expertinos::mrta_vc::tasks::Task task) 
{
    /*for (int i = 0; i < unallocated_tasks_.size(); i++)
    {
        if(task.equals(unallocated_tasks_.at(i)))
        {
            return;
        }
  }*/
    unallocated_tasks_.push(task);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::add(unifei::expertinos::mrta_vc::agents::Robot robot) 
{
    /*for (int i = 0; i < available_robots_.size(); i++)
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
  }*/
    std::list<unifei::expertinos::mrta_vc::agents::Robot>::iterator it = available_robots_.begin();
    while (it != available_robots_.end())
    {
        if(robot.equals(*it))
        {
            (*it).setLastBeaconTimestamp(robot.getLastBeaconTimestamp());
            (*it).setLocation(robot.getLocation());
            return;
        }
        ++it;
    }
    it = busy_robots_.begin();
    while (it != busy_robots_.end())
    {
        if(robot.equals(*it))
        {
            (*it).setLastBeaconTimestamp(robot.getLastBeaconTimestamp());
            (*it).setLocation(robot.getLocation());
            return;
        }
        ++it;
    }
    available_robots_.push_back(robot);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::add(unifei::expertinos::mrta_vc::agents::User user)
{
    ROS_INFO("TENTANDO LOGAR");
    std::list<unifei::expertinos::mrta_vc::agents::User>::iterator it = logged_users_.begin();
    while (it != logged_users_.end())
    {
        if(user.equals(*it))
        {
            (*it).setLastBeaconTimestamp(user.getLastBeaconTimestamp());
            (*it).setLocation(user.getLocation());
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
    /*for (int i = 0; i < unallocated_tasks_.size(); i++)
    {
        if(task.equals(unallocated_tasks_.at(i)))
        {
            unallocated_tasks_.erase(unallocated_tasks_.begin() + i);
            return;
        }
  }*/
    //unallocated_tasks_.pop(task);
    unallocated_tasks_.push(task);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::remove(unifei::expertinos::mrta_vc::agents::Robot robot) 
{
    /*for (int i = 0; i < available_robots_.size(); i++)
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
  }*/
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
 *	IMPLEMENTAR
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
    unifei::expertinos::mrta_vc::agents::Robot best_robot;
    double best_utility = 0.0;

    if (available_robots_.size() == 0)
    {
        return best_team;
    }
    std::list<unifei::expertinos::mrta_vc::agents::Robot>::iterator it = available_robots_.begin();
    while (it != available_robots_.end())
    {
        //ROS_INFO("Looking for a good robot");
        unifei::expertinos::mrta_vc::agents::Robot robot = (*it);
        if (robot.getUtility(task) > best_utility)
        {
            //ROS_INFO("achei um robo q pode realizar");
            best_utility = robot.getUtility(task);
            best_robot = robot;
        }
        ++it;
    }
    if (best_utility != 0.0)
    {
        best_team.push_back(best_robot);
    }
    return best_team;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::allocate(unifei::expertinos::mrta_vc::tasks::Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots)
{
    transfer(task);
    transfer(robots);

    unifei::expertinos::mrta_vc::tasks::Allocation allocation(task);
    allocation.allocate(robots);
    allocations_.push_back(allocation);

    //std::cout << "Number of allocations " << allocations_.size() << "\n";
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::transfer(unifei::expertinos::mrta_vc::tasks::Task task)
{
    allocated_tasks_.push_back(task);
    unallocated_tasks_.pop();
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::transfer(unifei::expertinos::mrta_vc::agents::Robot robot)
{
    std::list<unifei::expertinos::mrta_vc::agents::Robot>::iterator it = available_robots_.begin();
    while (it != available_robots_.end())
    {
        if (robot == (*it))
        {
            available_robots_.remove(robot);
            busy_robots_.push_back(robot);
            return;
        }
        ++it;
    }
    busy_robots_.remove(robot);
    available_robots_.push_back(robot);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::transfer(std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots)
{
    for (int i = 0; i < robots.size(); i++)
    {
        unifei::expertinos::mrta_vc::agents::Robot robot = robots.at(i);
        robot.setLastBeaconTimestamp();
        transfer(robot);
    }
}

/**
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::updateUnallocatedTasks()
{
    std::vector<unifei::expertinos::mrta_vc::tasks::Task> unallocated_tasks;
    for (int i = 0; i < unallocated_tasks_.size(); i++)
    {
        unifei::expertinos::mrta_vc::tasks::Task task = unallocated_tasks_.top();
        std::vector<unifei::expertinos::mrta_vc::agents::Robot> best_team = getBestTeam(task);
        if (best_team.size() != 0)
        {
            unifei::expertinos::mrta_vc::agents::Robot print_robot = best_team.at(0);
            ROS_INFO("Vou alocar! Task: %s para robo: %s", task.getName().c_str(), print_robot.getHostname().c_str());
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
 *
 */
void unifei::expertinos::mrta_vc::system::AllocationManager::updateAllocations(unifei::expertinos::mrta_vc::tasks::Allocation allocation)
{
    allocations_.remove(allocation);
    if (!allocation.wasEvaluated())
    {
        allocations_.push_back(allocation);
        return;
    }
    transfer(allocation.getRobots());
}
