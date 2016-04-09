/**
 *  Allocation.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 08/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/Allocation.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::Allocation(Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots, TaskStateEnum state, TaskSatisfactionEnum satisfaction) : task_(task), robots_(robots)
{
	state_ = state;
	satisfaction_ = satisfaction;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::Allocation(Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots, TaskStateEnum state, TaskSatisfactionEnum satisfaction, ros::Time allocation_timestamp, ros::Time start_timestamp, ros::Time end_timestamp) : task_(task), robots_(robots)
{
	state_ = state;
	satisfaction_ = satisfaction;
	allocation_timestamp_ = allocation_timestamp;
	setStartTimestamp(start_timestamp);
	setEndTimestamp(end_timestamp);
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::Allocation(const ::mrta_vc::Allocation::ConstPtr& allocation_msg) : task_(allocation_msg->task)
{
	for (int i = 0; i < allocation_msg->robots.size(); i++)
	{
		unifei::expertinos::mrta_vc::agents::Robot robot(allocation_msg->robots.at(i));
		robots_.push_back(robot);
	}
	state_ = TaskStates::toEnumerated(allocation_msg->state);
	satisfaction_ = TaskSatisfactions::toEnumerated(allocation_msg->satisfaction);
	allocation_timestamp_ = allocation_msg->allocation_timestamp;
	setStartTimestamp(allocation_msg->start_timestamp);
	setEndTimestamp(allocation_msg->end_timestamp);
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::Allocation(::mrta_vc::Allocation allocation_msg) : task_(allocation_msg.task)
{
	for (int i = 0; i < allocation_msg.robots.size(); i++)
	{
		unifei::expertinos::mrta_vc::agents::Robot robot(allocation_msg.robots.at(i));
		robots_.push_back(robot);
	}
	state_ = TaskStates::toEnumerated(allocation_msg.state);
	satisfaction_ = TaskSatisfactions::toEnumerated(allocation_msg.satisfaction);
	allocation_timestamp_ = allocation_msg.allocation_timestamp;
	setStartTimestamp(allocation_msg.start_timestamp);
	setEndTimestamp(allocation_msg.end_timestamp);	
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::~Allocation() 
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Task unifei::expertinos::mrta_vc::tasks::Allocation::getTask() 
{
	return task_;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::agents::Robot> unifei::expertinos::mrta_vc::tasks::Allocation::getRobots() 
{
	return robots_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::TaskStateEnum unifei::expertinos::mrta_vc::tasks::Allocation::getState() 
{
	return state_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum unifei::expertinos::mrta_vc::tasks::Allocation::getSatisfaction() 
{
	return satisfaction_;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::tasks::Allocation::getAllocationTimestamp() 
{
	return allocation_timestamp_;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::tasks::Allocation::getStartTimestamp() 
{
	return start_timestamp_;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::tasks::Allocation::getEndTimestamp() 
{
	return end_timestamp_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasAllocated() 
{
	return state_ != unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasAccepted() 
{
	return wasAllocated() && state_ != unifei::expertinos::mrta_vc::tasks::states::WAITING_ACCEPTATION;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::isExecuting() 
{
	return state_ == unifei::expertinos::mrta_vc::tasks::states::EXECUTING;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasSucceeded() 
{
	return state_ == unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasAborted() 
{
	return state_ == unifei::expertinos::mrta_vc::tasks::states::ABORTED;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasCancelled() 
{
	return state_ == unifei::expertinos::mrta_vc::tasks::states::CANCELLED;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::addRobot(unifei::expertinos::mrta_vc::agents::Robot robot) {
	for (int i = 0; i < robots_.size(); i++){
		if(robot.equals(robots_.at(i))){
			return;
		}
	}	
	robots_.push_back(robot);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::removeRobot(unifei::expertinos::mrta_vc::agents::Robot robot) {
	for (int i = 0; i < robots_.size(); i++){
		if(robot.equals(robots_.at(i))){
			robots_.erase(robots_.begin() + i);
			return;
		}
	}
}

/**
 * VERIFICAR SE HÁ NECESSIDADE DE FAZER O CONTROLE DOS ESTADOS
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setState(unifei::expertinos::mrta_vc::tasks::TaskStateEnum state)
{
	if (state_ != unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED && state_ != unifei::expertinos::mrta_vc::tasks::states::CANCELLED && state_ != unifei::expertinos::mrta_vc::tasks::states::ABORTED)
	{
		state_ = state;
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setSatisfaction(unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum satisfaction)
{
	satisfaction_ = satisfaction;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setAllocationTimestamp(ros::Time allocation_timestamp)
{
	allocation_timestamp_ = allocation_timestamp;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setStartTimestamp(ros::Time start_timestamp)
{
	if (allocation_timestamp_.isValid() && !end_timestamp_.isValid() && allocation_timestamp_ < start_timestamp) 
	{
		start_timestamp_ = start_timestamp;
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setEndTimestamp(ros::Time end_timestamp)
{
	if (start_timestamp_.isValid() && start_timestamp_ < end_timestamp)
	{
		end_timestamp_ = end_timestamp;
	}
}

/**
 *
 */
::mrta_vc::Allocation unifei::expertinos::mrta_vc::tasks::Allocation::toMsg()
{
	::mrta_vc::Allocation allocation_msg;
	allocation_msg.task = task_.toMsg();
	for(int i = 0; i < robots_.size(); i++) {
		allocation_msg.robots.push_back(robots_.at(i).toMsg());
	}
	allocation_msg.state = unifei::expertinos::mrta_vc::tasks::TaskStates::toCode(state_);
	allocation_msg.satisfaction = unifei::expertinos::mrta_vc::tasks::TaskSatisfactions::toCode(satisfaction_);
	allocation_msg.allocation_timestamp = allocation_timestamp_;
	allocation_msg.start_timestamp = start_timestamp_;
	allocation_msg.end_timestamp = end_timestamp_;
	return allocation_msg;
}

/**
 * IMPLEMENTAR
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::allocate(std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots)
{
	setAllocationTimestamp(ros::Time::now()); 
}

/**
 * IMPLEMENTAR
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::start()
{
	setStartTimestamp(ros::Time::now());
}

/**
 * IMPLEMENTAR
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::end()
{
	setEndTimestamp(ros::Time::now());
}

/**
 * IMPLEMENTAR
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::abort()
{
	setState(unifei::expertinos::mrta_vc::tasks::states::ABORTED);
}

/**
 * IMPLEMENTAR
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::cancel()
{
	setState(unifei::expertinos::mrta_vc::tasks::states::CANCELLED);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::equals(unifei::expertinos::mrta_vc::tasks::Allocation allocation)
{
	return task_.equals(allocation.task_);
}

/**
 * IMPLEMENTAR, talvez relacionado ao deadline da tarefa e ao nível de prioridade
 */
int unifei::expertinos::mrta_vc::tasks::Allocation::compareTo(unifei::expertinos::mrta_vc::tasks::Allocation allocation)
{
	return 0; /////////////
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::operator==(const unifei::expertinos::mrta_vc::tasks::Allocation& allocation)
{
	return task_ == allocation.task_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::operator!=(const unifei::expertinos::mrta_vc::tasks::Allocation& allocation)
{
	return task_ != allocation.task_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::operator=(const unifei::expertinos::mrta_vc::tasks::Allocation& allocation)
{
	
}
