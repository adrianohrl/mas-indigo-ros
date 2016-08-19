/**
 *  Task.cpp
 *
 *  Version: 1.2.4
 *  Created on: 06/04/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/Task.h"

namespace mas
{
	namespace tasks
	{

		/**
		 *
		 */
		Task::Task() : sender_(mas_msgs::Agent()), receiver_(mas_msgs::Agent())
		{
			id_ = 0;
			name_ = "";	
			description_ = "";
			priority_ = TaskPriorities::getDefault();
		}

		/**
		 *
		 */
		Task::Task(int id, std::string name, std::string description, std::vector<Skill> desired_skills, agents::User user, agents::Person sender, agents::Person receiver, ros::Time deadline, priorities::TaskPriorityEnum priority) : desired_skills_(desired_skills), user_(user), sender_(sender), receiver_(receiver)
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
		Task::Task(int id, std::string name, std::string description, std::vector<Skill> desired_skills, agents::User user, agents::Person sender, agents::Person receiver, ros::Duration duration, priorities::TaskPriorityEnum priority) : desired_skills_(desired_skills), user_(user), sender_(sender), receiver_(receiver)
		{
		  id_ = id;
		  name_ = name;
		  description_ = description;
		  priority_ = priority;
		  deadline_ = ros::Time::now() + duration;
		}

		/**
		 *
		 */
		Task::Task(const mas_msgs::Task::ConstPtr& task_msg) : user_(task_msg->user), sender_(task_msg->sender), receiver_(task_msg->receiver)
		{
			id_ = task_msg->id;	
			name_ = task_msg->name;
			description_ = task_msg->description;	
			for(int i = 0; i < task_msg->desired_skills.size(); i++) 
			{
				Skill skill(task_msg->desired_skills.at(i)); 		
				desired_skills_.push_back(skill);	
			}	
			priority_ = TaskPriorities::toEnumerated(task_msg->priority);
			deadline_ = task_msg->deadline;
		}

		/**
		 *
		 */
		Task::Task(mas_msgs::Task task_msg) : user_(task_msg.user), sender_(task_msg.sender), receiver_(task_msg.receiver)
		{
			id_ = task_msg.id;	
			name_ = task_msg.name;
			description_ = task_msg.description;
			for(int i = 0; i < task_msg.desired_skills.size(); i++) 
			{
				Skill skill(task_msg.desired_skills.at(i)); 		
				desired_skills_.push_back(skill);	
			}	
			priority_ = TaskPriorities::toEnumerated(task_msg.priority);	
			deadline_ = task_msg.deadline;
		}

		/**
		 *
		 */
		Task::~Task() 
		{
		}

		/**
		 *
		 */
		int Task::getId() 
		{
			return id_;
		}

		/**
		 *
		 */
		std::string Task::getName() 
		{
			return name_;
		}

		/**
		 *
		 */
		std::string Task::getDescription() 
		{
			return description_;
		}

		/**
		 *
		 */
		std::vector<Skill> Task::getDesiredSkills() 
		{
			return desired_skills_;
		}

		/**
		 *
		 */
		agents::User Task::getUser()
		{
		  return user_;
		}

		/**
		 *
		 */
		agents::Person Task::getSender()
		{
		  return sender_;
		}

		/**
		 *
		 */
		agents::Person Task::getReceiver() 
		{
			return receiver_;
		}

		/**
		 *
		 */
		priorities::TaskPriorityEnum Task::getPriority() 
		{
			return priority_;
		}

		/**
		 *
		 */
		ros::Time Task::getDeadline() 
		{
			return deadline_;
		}

		/**
		 *
		 */
		void Task::setId(int id) 
		{
			id_ = id;
		}

		/**
		 *
		 */
		void Task::setName(std::string name) 
		{
			name_ = name;
		}

		/**
		 *
		 */
		void Task::setDescription(std::string description) 
		{
			description_ = description;
		}

		/**
		 *
		 */
		void Task::addSkill(Skill skill) 
		{
			for (int i = 0; i < desired_skills_.size(); i++)
			{
				if(skill.equals(desired_skills_.at(i)))
				{
					return;
				}
			}	
			desired_skills_.push_back(skill);
		}

		/**
		 *
		 */
		void Task::removeSkill(Skill skill) 
		{
			for (int i = 0; i < desired_skills_.size(); i++)
			{
				if(skill.equals(desired_skills_.at(i)))
				{
					desired_skills_.erase(desired_skills_.begin() + i);
					return;
				}
			}
		}

		/**
		 *
		 */
		void Task::setUser(agents::User user)
		{
		  user_ = user;
		}

		/**
		 *
		 */
		void Task::setSender(agents::Person sender)
		{
		  sender_ = sender;
		}

		/**
		 *
		 */
		void Task::setReceiver(agents::Person receiver) 
		{
			receiver_ = receiver;
		}

		/**
		 *
		 */
		void Task::setPriority(priorities::TaskPriorityEnum priority) 
		{
			priority_ = priority;
		}

		/**
		 *
		 */
		void Task::setDeadline(ros::Time deadline)
		{
		  deadline_ = deadline;
		}

		/**
		 *
		 */
		void Task::setDeadline(ros::Duration duration)
		{
		  deadline_ = ros::Time::now() + duration;
		}

		/**
		 *
		 */
		bool Task::isExpired()
		{
			return deadline_.toSec() != 0.0 && deadline_ < ros::Time::now();
		}

		/**
		 *
		 */
		bool Task::isInvolved(agents::Person person)
		{
			return person == user_ || person == receiver_ || person == sender_;
		}

		/**
		 *
		 */
		mas_msgs::Task Task::toMsg() 
		{
			mas_msgs::Task task_msg;
			task_msg.id = id_;
			task_msg.name = name_;
			task_msg.description = description_;
		  for (int i = 0; i < desired_skills_.size(); i++)
			{
				task_msg.desired_skills.push_back(desired_skills_.at(i).toMsg());
			}
		  task_msg.user = user_.toMsg();
			task_msg.sender = sender_.toMsg();
			task_msg.receiver = receiver_.toMsg();
			task_msg.priority = TaskPriorities::toCode(priority_);
			task_msg.deadline = deadline_;
			return task_msg;
		}


		/**
		 *
		 */
		std::string Task::toString()
		{
		  std::stringstream desired_skills_ss;
		  for (int i = 0; i < desired_skills_.size(); i++)
		  {
		    if (i != 0)
		    {
			 desired_skills_ss << ", ";
		    }
		    desired_skills_ss << i << " " << desired_skills_.at(i).toString();
			}
			return "task: {name: " + name_ +
					", description: " + description_ +
					", skills: {" + desired_skills_ss.str() +
					"}, user: " + user_.toString() +
					", sender: " + sender_.toString() +
					", receiver: " + receiver_.toString() +
					", priority: " + TaskPriorities::toString(priority_) +
					", deadline: " + utilities::TimeManipulator::toString(deadline_) +
					"}";
		}

		/**
		 *
		 */
		int Task::compareTo(Task task)
		{
			return 10 * TaskPriorities::compare(priority_, task.priority_)
					+ 2 * user_.compareTo(task.user_)
					+ sender_.compareTo(task.sender_)
					+ receiver_.compareTo(task.receiver_);
		}

		/**
		 *
		 */
		bool Task::equals(Task task)
		{
			return operator==(task);
		}

		/**
		 *
		 */
		bool Task::operator==(const Task& task)
		{
			return id_ == task.id_;
		}

		/**
		 *
		 */
		bool Task::operator!=(const Task& task) 
		{
			return !operator==(task);
		}

		/**
		 *
		 */
		void Task::operator=(const Task &task)
		{ 
			id_ = task.id_;
			name_ = task.name_;
			description_ = task.description_;
			desired_skills_ = task.desired_skills_;
		  user_ = task.user_;
			sender_ = task.sender_;
			receiver_ = task.receiver_;
			priority_ = task.priority_;
			deadline_ = task.deadline_;
		}
		
	}
}
