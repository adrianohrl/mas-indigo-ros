/**
 *  Task.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 06/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/Task.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Task::Task() : sender_(::mrta_vc::Agent()), receiver_(::mrta_vc::Agent())
{
	id_ = 0;
	name_ = "";	
	description_ = "";
	priority_ = unifei::expertinos::mrta_vc::tasks::TaskPriorities::getDefault();
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Task::Task(int id, std::string name, std::string description, std::vector<unifei::expertinos::mrta_vc::tasks::Skill> desired_skills, unifei::expertinos::mrta_vc::agents::VoiceCommander sender, unifei::expertinos::mrta_vc::agents::Person receiver, ros::Time deadline, unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum priority) : desired_skills_(desired_skills), sender_(sender), receiver_(receiver)
{
	id_ = id;
	name_ = name;
	description_ = description;	
	priority_ = priority;
	deadline_ = deadline;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Task::Task(const ::mrta_vc::Task::ConstPtr& task_msg) : sender_(task_msg->sender), receiver_(task_msg->receiver)
{
	id_ = task_msg->id;	
	name_ = task_msg->name;
	description_ = task_msg->description;	
	for(int i = 0; i < task_msg->desired_skills.size(); i++) {
		Skill skill(task_msg->desired_skills.at(i)); 		
		desired_skills_.push_back(skill);	
	}	
	priority_ = unifei::expertinos::mrta_vc::tasks::TaskPriorities::toEnumerated(task_msg->priority);
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Task::Task(::mrta_vc::Task task_msg) : sender_(task_msg.sender), receiver_(task_msg.receiver)
{
	id_ = task_msg.id;	
	name_ = task_msg.name;
	description_ = task_msg.description;
	for(int i = 0; i < task_msg.desired_skills.size(); i++) {
		Skill skill(task_msg.desired_skills.at(i)); 		
		desired_skills_.push_back(skill);	
	}	
	priority_ = unifei::expertinos::mrta_vc::tasks::TaskPriorities::toEnumerated(task_msg.priority);	
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Task::~Task() 
{
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::Task::getId() 
{
	return id_;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::tasks::Task::getName() 
{
	return name_;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::tasks::Task::getDescription() 
{
	return description_;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::Skill> unifei::expertinos::mrta_vc::tasks::Task::getDesiredSkills() 
{
	return desired_skills_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::VoiceCommander unifei::expertinos::mrta_vc::tasks::Task::getSender() 
{
	return sender_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person unifei::expertinos::mrta_vc::tasks::Task::getReceiver() 
{
	return receiver_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum unifei::expertinos::mrta_vc::tasks::Task::getPriority() 
{
	return priority_;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::tasks::Task::getDeadline() 
{
	return deadline_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Task::setId(int id) 
{
	id_ = id;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Task::setName(std::string name) 
{
	name_ = name;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Task::setDescription(std::string description) 
{
	description_ = description;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Task::addSkill(unifei::expertinos::mrta_vc::tasks::Skill skill) {
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
void unifei::expertinos::mrta_vc::tasks::Task::removeSkill(unifei::expertinos::mrta_vc::tasks::Skill skill) {
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
void unifei::expertinos::mrta_vc::tasks::Task::setSender(unifei::expertinos::mrta_vc::agents::VoiceCommander sender) 
{
	sender_ = sender;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Task::setReceiver(unifei::expertinos::mrta_vc::agents::Person receiver) 
{
	receiver_ = receiver;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Task::setPriority(unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum priority) 
{
	priority_ = priority;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Task::setDeadline(ros::Time deadline) 
{
	deadline_ = deadline;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Task::hasExpired() 
{
	return deadline_ < ros::Time::now();
}

/**
 *
 */
::mrta_vc::Task unifei::expertinos::mrta_vc::tasks::Task::toMsg() 
{
	::mrta_vc::Task task_msg;
	task_msg.id = id_;
	task_msg.name = name_;
	task_msg.description = description_;
	for(int i = 0; i < desired_skills_.size(); i++) {
		task_msg.desired_skills.push_back(desired_skills_.at(i).toMsg());
	}
	task_msg.sender = sender_.toMsg();
	task_msg.receiver = receiver_.toMsg();
	task_msg.priority = unifei::expertinos::mrta_vc::tasks::TaskPriorities::toCode(priority_);
	task_msg.deadline = deadline_;
	return task_msg;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Task::equals(unifei::expertinos::mrta_vc::tasks::Task task) 
{
	return name_ == task.name_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Task::operator==(const unifei::expertinos::mrta_vc::tasks::Task& task)
{
	return name_ == task.name_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Task::operator!=(const unifei::expertinos::mrta_vc::tasks::Task& task) 
{
	return name_ != task.name_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Task::operator=(const unifei::expertinos::mrta_vc::tasks::Task &task)
{ 
	id_ = task.id_;
	name_ = task.name_;
	description_ = task.description_;
	desired_skills_ = task.desired_skills_;
	sender_ = task.sender_;
	receiver_ = task.receiver_;
	priority_ = task.priority_;
	deadline_ = task.deadline_;
}
