/**
 *  SystemManagerNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemManagerNode.h"

/**
 * Constructor
 */
mrta_vc::SystemManagerNode::SystemManagerNode(ros::NodeHandle nh) : nh_(nh)
{
    robots_sub_ = nh_.subscribe("/robots", 100, &mrta_vc::SystemManagerNode::robotsCallback, this);
    tasks_sub_ = nh_.subscribe("/tasks", 100, &mrta_vc::SystemManagerNode::tasksCallback, this);
    users_sub_ = nh_.subscribe("/users", 100, &mrta_vc::SystemManagerNode::usersCallback, this);
    allocation_sub_ = nh_.subscribe("/running_allocations", 100, &mrta_vc::SystemManagerNode::allocationsCallback, this);
    allocation_pub_ = nh.advertise<mrta_vc::Allocation>("/allocations", 1);
    manager_state_pub_ = nh.advertise<mrta_vc::ManagerState>("/manager_state", 1);
    robots_timer_ = nh_.createTimer(ros::Duration(.75 * ROBOT_BEACON_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::robotsTimerCallback, this);
    tasks_timer_ = nh_.createTimer(ros::Duration(TASK_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::tasksTimerCallback, this);
    users_timer_ = nh_.createTimer(ros::Duration(.75 * USER_BEACON_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::usersTimerCallback, this);
    allocation_timer_ = nh_.createTimer(ros::Duration(.75 * ALLOCATION_INTERVAL_DURATION), &mrta_vc::SystemManagerNode::allocationTimerCallback, this);
}

/**
 * Destructor
 */
mrta_vc::SystemManagerNode::~SystemManagerNode()
{
    robots_timer_.stop();
    tasks_timer_.stop();
    users_timer_.stop();
    robots_sub_.shutdown();
    tasks_sub_.shutdown();
    users_sub_.shutdown();
}

/**
 *
 */
void mrta_vc::SystemManagerNode::spin() 
{
    ROS_INFO("System Manager Node is up and running!!!");
    ros::Rate loop_rate(10.0);
    while (nh_.ok())
    {
        managerStatePublish();
        ros::spinOnce();
        /*std::cout << "Number of available robots: " << unifei::expertinos::mrta_vc::system::AllocationManager::getAvailableRobots().size() << "\n";
        std::cout << "Number of busy robots: " << unifei::expertinos::mrta_vc::system::AllocationManager::getBusyRobots().size() << "\n";
        std::cout << "Number of available tasks: " << unifei::expertinos::mrta_vc::system::AllocationManager::getUnallocatedTasks().size() << "\n";
        if (unifei::expertinos::mrta_vc::system::AllocationManager::getUnallocatedTasks().size() != 0)
        {
            unifei::expertinos::mrta_vc::tasks::Task task = unifei::expertinos::mrta_vc::system::AllocationManager::getUnallocatedTasks().top();
            ROS_INFO("First task of the priority queue: %s", task.getName().c_str());
        }*/
        loop_rate.sleep();
    }
}

/**
 *
 */
void mrta_vc::SystemManagerNode::robotsCallback(const mrta_vc::Agent::ConstPtr& robot_msg)
{
    unifei::expertinos::mrta_vc::agents::Robot robot(robot_msg);
    robot.setLastBeaconTimestamp();
    unifei::expertinos::mrta_vc::system::AllocationManager::add(robot);
}

/**
 *
 */
void mrta_vc::SystemManagerNode::tasksCallback(const mrta_vc::Task::ConstPtr& task_msg)
{
    unifei::expertinos::mrta_vc::tasks::Task task(task_msg);
    ROS_INFO("%s", task.toString().c_str());
    //ROS_WARN("now: %f s (%f s)", ros::Time::now().toSec(), task.getDeadline().toSec());
    unifei::expertinos::mrta_vc::system::AllocationManager::add(task);
    //unifei::expertinos::mrta_vc::tasks::Task topTask = unifei::expertinos::mrta_vc::system::AllocationManager::getUnallocatedTasks().top();
    //ROS_INFO("%s", topTask.toString().c_str());
}

/**
 *
 */
void mrta_vc::SystemManagerNode::usersCallback(const mrta_vc::Agent::ConstPtr& user_msg)
{
    unifei::expertinos::mrta_vc::agents::User user(user_msg);
    user.setLastBeaconTimestamp();
    unifei::expertinos::mrta_vc::system::AllocationManager::add(user);
}

/**
 *
 */
void mrta_vc::SystemManagerNode::allocationsCallback(const mrta_vc::Allocation::ConstPtr& allocation_msg)
{
    unifei::expertinos::mrta_vc::tasks::Allocation allocation(allocation_msg);
    unifei::expertinos::mrta_vc::system::AllocationManager::updateAllocations(allocation);
}

/**
 *
 */
void mrta_vc::SystemManagerNode::robotsTimerCallback(const ros::TimerEvent& event)
{
    unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedRobots();
}

/**
 *
 */
void mrta_vc::SystemManagerNode::tasksTimerCallback(const ros::TimerEvent& event)
{
    unifei::expertinos::mrta_vc::system::AllocationManager::updateUnallocatedTasks();
}

/**
 *
 */
void mrta_vc::SystemManagerNode::usersTimerCallback(const ros::TimerEvent& event)
{
    unifei::expertinos::mrta_vc::system::AllocationManager::updateLoggedUsers();
}

/**
 *
 */
void mrta_vc::SystemManagerNode::allocationTimerCallback(const ros::TimerEvent& event)
{
    std::list<unifei::expertinos::mrta_vc::tasks::Allocation> allocations = unifei::expertinos::mrta_vc::system::AllocationManager::getAllocations();

    std::list<unifei::expertinos::mrta_vc::tasks::Allocation>::iterator it = allocations.begin();
    while (it != allocations.end())
    {
        unifei::expertinos::mrta_vc::tasks::Allocation allocation = (*it);
        if (!allocation.wasAllocated())
        {
            //Como fazer para publicar apenas uma vez
            allocation_pub_.publish(allocation.toMsg());
        }
        ++it;
    }
}

/**
 *
 */
void mrta_vc::SystemManagerNode::managerStatePublish()
{
    ::mrta_vc::ManagerState manager_state_msg;
    manager_state_msg.number_of_unallocated_tasks = unifei::expertinos::mrta_vc::system::AllocationManager::getUnallocatedTasks().size();
    manager_state_msg.number_of_allocated_tasks = unifei::expertinos::mrta_vc::system::AllocationManager::getAllocatedTasks().size();
    manager_state_msg.number_of_available_robots = unifei::expertinos::mrta_vc::system::AllocationManager::getAvailableRobots().size();
    manager_state_msg.number_of_busy_robots = unifei::expertinos::mrta_vc::system::AllocationManager::getBusyRobots().size();
    manager_state_msg.number_of_logged_users = unifei::expertinos::mrta_vc::system::AllocationManager::getLoggedUsers().size();
    manager_state_msg.number_of_allocations = unifei::expertinos::mrta_vc::system::AllocationManager::getAllocations().size();
    manager_state_pub_.publish(manager_state_msg);
}
