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
	allocation_timestamp_ = NULL;
	start_timestamp_ = NULL;
	end_timestamp_ = NULL;
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
Task unifei::expertinos::mrta_vc::tasks::Allocation::getTask() 
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
TaskStateEnum unifei::expertinos::mrta_vc::tasks::Allocation::getState() 
{
	return state_;
}

/**
 *
 */
TaskSatisfactionEnum unifei::expertinos::mrta_vc::tasks::Allocation::getSatisfaction() 
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
	return wasAllocated() && state_ != unifei::expertinos::mrta_vc::tasks::states::WAITING_ACCEPTANCE;
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
void unifei::expertinos::mrta_vc::tasks::Allocation::setState(TaskStateEnum state)
{
	if (state_ != unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED && state_ != unifei::expertinos::mrta_vc::tasks::states::CANCELLED && state_ != unifei::expertinos::mrta_vc::tasks::states::ABORTED)
	{
		state_ = state;
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setSatisfaction(TaskSatisfactionEnum satisfaction)
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
void unifei::expertinos::mrta_vc::tasks::Allocation::setEndTimestamp(ros::Timestamp end_timestamp)
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
	allocation_msg.state = TaskStates::toCode(state_);
	allocation_msg.satisfaction = TaskSatisfactions::toCode(satisfaction_);
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
bool unifei::expertinos::mrta_vc::tasks::Allocation::equals(Allocation allocation)
{
	return task_.equals(allocation.task_);
}

/**
 * IMPLEMENTAR, talvez relacionado ao deadline da tarefa e ao nível de prioridade
 */
int compareTo(Allocation allocation)
{
	return 0; /////////////
}

/**
 *
 */
bool operator==(const Allocation& allocation)
{
	return task_ == allocation->task_;
}

/**
 *
 */
bool operator!=(const Allocation& allocation)
{
	return task_ != allocation->task_;
}

/**
 *
 */
void operator=(const Allocation& allocation)
{
	
}




/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::addSkill(unifei::expertinos::mrta_vc::tasks::Skill skill) {
	for (int i = 0; i < desired_skills_.size(); i++){
		if(skill.equals(desired_skills_.at(i))){
			return;
		}
	}	
	desired_skills_.push_back(skill);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::removeSkill(unifei::expertinos::mrta_vc::tasks::Skill skill) {
	for (int i = 0; i < desired_skills_.size(); i++){
		if(skill.equals(desired_skills_.at(i))){
			desired_skills_.erase(desired_skills_.begin() + i);
			return;
		}
	}
}




/**
 *
 */
::mrta_vc::Allocation unifei::expertinos::mrta_vc::tasks::Allocation::toMsg() 
{
	::mrta_vc::Allocation task_msg;
	task_msg.id = id_;
	task_msg.name = name_;
	task_msg.description = description_;
	for(int i = 0; i < desired_skills_.size(); i++) {
		task_msg.desired_skills.push_back(desired_skills_.at(i).toMsg());
	}
	task_msg.sender = sender_.toMsg();
	task_msg.receiver = receiver_.toMsg();
	task_msg.priority = unifei::expertinos::mrta_vc::tasks::AllocationPriorities::toCode(priority_);
	return task_msg;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::equals(Allocation task) 
{
	return name_ == task.name_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::operator==(const Allocation& task)
{
	return name_ == task.name_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::operator!=(const Allocation& task) 
{
	return name_ != task.name_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::operator=(const Allocation &task)
{ 
	id_ = task.id_;
	name_ = task.name_;
	description_ = task.description_;
	desired_skills_ = task.desired_skills_;
	sender_ = task.sender_;
	receiver_ = task.receiver_;
	priority_ = task.priority_;
}
