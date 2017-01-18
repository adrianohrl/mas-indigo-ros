/**
 *  This header file defines the Task class.
 *
 *  Version: 1.4.0
 *  Created on: 04/08/2015
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASKS_TASK_H_
#define _TASKS_TASK_H_

#include <queue>
#include <vector>
#include <ros/ros.h>
#include <mas_msgs/Task.h>
#include "mas/agents/user.h"
#include "mas/tasks/skill.h"
#include "mas/tasks/task_priorities.h"
#include "utilities/time_manipulator.h"

namespace mas
{
namespace tasks
{
class Task
{
public:
  Task();
  Task(int id, std::string name, std::string description,
       const std::vector<Skill*>& skills, agents::User* user,
       agents::Person* sender, agents::Person* receiver,
       const ros::Time& deadline,
       TaskPriorityEnum priority = TaskPriorities::getDefault());
  Task(int id, std::string name, std::string description,
       const std::vector<Skill*>& skills, agents::User* user,
       agents::Person* sender, agents::Person* receiver,
       const ros::Duration& duration,
       TaskPriorityEnum priority = TaskPriorities::getDefault());
  Task(const mas_msgs::Task::ConstPtr& task_msg);
  Task(const mas_msgs::Task& task_msg);
  ~Task();

  int getId() const;
  std::string getName() const;
  std::string getDescription() const;
  std::vector<Skill*> getSkills() const;
  agents::User* getUser() const;
  agents::Person* getSender() const;
  agents::Person* getReceiver() const;
  TaskPriorityEnum getPriority() const;
  ros::Time getDeadline() const;
  void setId(int id);
  void setName(std::string name);
  void setDescription(std::string description);
  void clearSkills();
  void setSkills(const std::vector<Skill*>& skills);
  void addSkill(Skill* skill);
  void removeSkill(const Skill& skill);
  void setUser(agents::User* user);
  void setSender(agents::Person* sender);
  void setReceiver(agents::Person* receiver);
  void setPriority(TaskPriorityEnum priority);
  void setDeadline(const ros::Time& deadline);
  void setDeadline(const ros::Duration& duration);
  bool isExpired() const;
  bool isInvolved(const agents::Person& person) const;
  virtual mas_msgs::Task to_msg() const;
  virtual std::string str() const;
  const char* c_str() const;
  void operator=(const Task& task);
  bool operator==(const Task& task) const;
  bool operator!=(const Task& task) const;
  int compareTo(const Task& task) const;

private:
  int id_;
  std::string name_;
  std::string description_;
  std::vector<Skill*> skills_;
  agents::User* user_;
  agents::Person* sender_;
  agents::Person* receiver_;
  TaskPriorityEnum priority_;
  ros::Time deadline_;
};

struct TaskComparator
{
  bool operator()(Task* task1, Task* task2) const
  {
    return task2->compareTo(*task1) > 0;
  }
};

typedef std::priority_queue<Task*, std::vector<Task*>, TaskComparator>
    TaskPriorityQueue;
}
}

#endif /* _TASKS_TASK_H_ */
