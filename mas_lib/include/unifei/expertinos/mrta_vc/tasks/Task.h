/**
 *  Task.h
 *
 *  Version: 1.2.2
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
#include "unifei/expertinos/mrta_vc/agents/User.h"
#include "unifei/expertinos/mrta_vc/tasks/Skill.h"
#include "unifei/expertinos/mrta_vc/tasks/TaskPriorities.h"
#include "unifei/expertinos/utilities/TimeManipulator.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace tasks
      {
				class Task 
				{
				public:
          Task();
          Task(int id, std::string name, std::string description, std::vector<Skill> desired_skills, unifei::expertinos::mrta_vc::agents::User user, unifei::expertinos::mrta_vc::agents::Person sender, unifei::expertinos::mrta_vc::agents::Person receiver, ros::Time deadline, TaskPriorityEnum priority = TaskPriorities::getDefault());
          Task(int id, std::string name, std::string description, std::vector<Skill> desired_skills, unifei::expertinos::mrta_vc::agents::User user, unifei::expertinos::mrta_vc::agents::Person sender, unifei::expertinos::mrta_vc::agents::Person receiver, ros::Duration duration, TaskPriorityEnum priority = TaskPriorities::getDefault());
					Task(const mas_msgs::Task::ConstPtr& task_msg);
					Task(mas_msgs::Task task_msg);		
					~Task();

					int getId();
					std::string getName();
					std::string getDescription();
					std::vector<unifei::expertinos::mrta_vc::tasks::Skill> getDesiredSkills();
          unifei::expertinos::mrta_vc::agents::User getUser();
          unifei::expertinos::mrta_vc::agents::Person getSender();
					unifei::expertinos::mrta_vc::agents::Person getReceiver();
					TaskPriorityEnum getPriority();
					ros::Time getDeadline();
					void setId(int id);
					void setName(std::string name);
					void setDescription(std::string description);
					void addSkill(Skill skill);
					void removeSkill(Skill skill);
          void setUser(unifei::expertinos::mrta_vc::agents::User user);
          void setSender(unifei::expertinos::mrta_vc::agents::Person sender);
					void setReceiver(unifei::expertinos::mrta_vc::agents::Person receiver);
          void setPriority(TaskPriorityEnum priority);
          void setDeadline(ros::Time deadline);
          void setDeadline(ros::Duration duration);
					bool isExpired();
					bool isInvolved(unifei::expertinos::mrta_vc::agents::Person person);
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
          unifei::expertinos::mrta_vc::agents::User user_;
          unifei::expertinos::mrta_vc::agents::Person sender_;
					unifei::expertinos::mrta_vc::agents::Person receiver_;
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
	}
}		

#endif /* TASKS_TASK_H_ */
