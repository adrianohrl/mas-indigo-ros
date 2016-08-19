/**
 *  Task.h
 *
 *  Version: 1.2.4
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASKS_TASK_H_
#define TASKS_TASK_H_

#include <queue>
#include <vector>
#include <ros/ros.h>
#include <mas_msgs/Task.h>
#include "mas/agents/User.h"
#include "mas/tasks/Skill.h"
#include "mas/tasks/TaskPriorities.h"
#include "utilities/TimeManipulator.h"

namespace mas 
{
	namespace tasks
	{
		class Task 
		{
		public:
			Task();
			Task(int id, std::string name, std::string description, std::vector<Skill> desired_skills, agents::User user, agents::Person sender, agents::Person receiver, ros::Time deadline, TaskPriorityEnum priority = TaskPriorities::getDefault());
			Task(int id, std::string name, std::string description, std::vector<Skill> desired_skills, agents::User user, agents::Person sender, agents::Person receiver, ros::Duration duration, TaskPriorityEnum priority = TaskPriorities::getDefault());
			Task(const mas_msgs::Task::ConstPtr& task_msg);
			Task(mas_msgs::Task task_msg);		
			~Task();

			int getId();
			std::string getName();
			std::string getDescription();
			std::vector<Skill> getDesiredSkills();
			agents::User getUser();
			agents::Person getSender();
			agents::Person getReceiver();
			TaskPriorityEnum getPriority();
			ros::Time getDeadline();
			void setId(int id);
			void setName(std::string name);
			void setDescription(std::string description);
			void addSkill(Skill skill);
			void removeSkill(Skill skill);
			void setUser(agents::User user);
			void setSender(agents::Person sender);
			void setReceiver(agents::Person receiver);
			void setPriority(TaskPriorityEnum priority);
			void setDeadline(ros::Time deadline);
			void setDeadline(ros::Duration duration);
			bool isExpired();
			bool isInvolved(agents::Person person);
			mas_msgs::Task toMsg();
			std::string toString();
			int compareTo(Task task);
			bool equals(Task task);
			bool operator==(const Task& task);
			bool operator!=(const Task& task);
			void operator=(const Task& task);

		private:
			int id_;
			std::string name_;
			std::string description_;
			std::vector<Skill> desired_skills_;
			agents::User user_;
			agents::Person sender_;
			agents::Person receiver_;
			TaskPriorityEnum priority_;
			ros::Time deadline_;

		};

		struct TaskComparator
		{
			bool operator()(Task task1, Task task2)
			{
				return task2.compareTo(task1) > 0;
			}
		};

		typedef std::priority_queue<Task, std::vector<Task>, TaskComparator> TaskPriorityQueue;
	}
}		

#endif /* TASKS_TASK_H_ */
