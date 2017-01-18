/**
 *  This source file implements the Task class.
 *
 *  Version: 1.4.0
 *  Created on: 06/04/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task.h"

namespace mas
{
namespace tasks
{

/**
 * @brief Task::Task
 */
Task::Task()
    : user_(NULL), sender_(NULL), receiver_(NULL), id_(0), name_(""),
      description_(""), priority_(TaskPriorities::getDefault())
{
}

/**
 * @brief Task::Task
 * @param id
 * @param name
 * @param description
 * @param skills
 * @param user
 * @param sender
 * @param receiver
 * @param deadline
 * @param priority
 */
Task::Task(int id, std::string name, std::string description,
           const std::vector<Skill*>& skills, agents::User* user,
           agents::Person* sender, agents::Person* receiver,
           const ros::Time& deadline, priorities::TaskPriorityEnum priority)
    : skills_(skills), user_(user), sender_(sender), receiver_(receiver),
      id_(id), name_(name), description_(description), priority_(priority),
      deadline_(deadline)
{
}

/**
 * @brief Task::Task
 * @param id
 * @param name
 * @param description
 * @param skills
 * @param user
 * @param sender
 * @param receiver
 * @param duration
 * @param priority
 */
Task::Task(int id, std::string name, std::string description,
           const std::vector<Skill*>& skills, agents::User* user,
           agents::Person* sender, agents::Person* receiver,
           const ros::Duration& duration, priorities::TaskPriorityEnum priority)
    : skills_(skills), user_(user), sender_(sender), receiver_(receiver),
      id_(id), name_(name), description_(description), priority_(priority),
      deadline_(ros::Time::now() + duration)
{
}

/**
 * @brief Task::Task
 * @param task_msg
 */
Task::Task(const mas_msgs::Task::ConstPtr& task_msg)
    : user_(new agents::User(task_msg->user)),
      sender_(new agents::Person(task_msg->sender)),
      receiver_(new agents::Person(task_msg->receiver)), id_(task_msg->id),
      name_(task_msg->name), description_(task_msg->description),
      priority_(TaskPriorities::toEnumerated(task_msg->priority)),
      deadline_(task_msg->deadline)
{
  for (int i(0); i < task_msg->skills.size(); i++)
  {
    Skill* skill = new Skill(task_msg->skills[i]);
    skills_.push_back(skill);
  }
}

/**
 * @brief Task::Task
 * @param task_msg
 */
Task::Task(const mas_msgs::Task& task_msg)
    : user_(new agents::User(task_msg.user)),
      sender_(new agents::Person(task_msg.sender)),
      receiver_(new agents::Person(task_msg.receiver)), id_(task_msg.id),
      name_(task_msg.name), description_(task_msg.description),
      priority_(TaskPriorities::toEnumerated(task_msg.priority)),
      deadline_(task_msg.deadline)
{
  for (int i(0); i < task_msg.skills.size(); i++)
  {
    Skill* skill = new Skill(task_msg.skills[i]);
    skills_.push_back(skill);
  }
}

/**
 * @brief Task::~Task
 */
Task::~Task()
{
  clearSkills();
  if (user_)
  {
    delete user_;
    user_ = NULL;
  }
  if (sender_)
  {
    delete sender_;
    sender_ = NULL;
  }
  if (receiver_)
  {
    delete receiver_;
    receiver_ = NULL;
  }
}

/**
 * @brief Task::getId
 * @return
 */
int Task::getId() const { return id_; }

/**
 * @brief Task::getName
 * @return
 */
std::string Task::getName() const { return name_; }

/**
 * @brief Task::getDescription
 * @return
 */
std::string Task::getDescription() const { return description_; }

/**
 * @brief Task::getDesiredSkills
 * @return
 */
std::vector<Skill*> Task::getSkills() const { return skills_; }

/**
 * @brief Task::getUser
 * @return
 */
agents::User* Task::getUser() const { return user_; }

/**
 * @brief Task::getSender
 * @return
 */
agents::Person* Task::getSender() const { return sender_; }

/**
 * @brief Task::getReceiver
 * @return
 */
agents::Person* Task::getReceiver() const { return receiver_; }

/**
 * @brief Task::getPriority
 * @return
 */
priorities::TaskPriorityEnum Task::getPriority() const { return priority_; }

/**
 * @brief Task::getDeadline
 * @return
 */
ros::Time Task::getDeadline() const { return deadline_; }

/**
 * @brief Task::setId
 * @param id
 */
void Task::setId(int id) { id_ = id; }

/**
 * @brief Task::setName
 * @param name
 */
void Task::setName(std::string name) { name_ = name; }

/**
 * @brief Task::setDescription
 * @param description
 */
void Task::setDescription(std::string description)
{
  description_ = description;
}

/**
 * @brief Task::clearSkills
 */
void Task::clearSkills()
{
  for (int i(0); i < skills_.size(); i++)
  {
    if (skills_[i])
    {
      delete skills_[i];
      skills_[i] = NULL;
    }
  }
  skills_.clear();
}

/**
 * @brief Task::setSkills
 * @param skills
 */
void Task::setSkills(const std::vector<Skill*>& skills)
{
  clearSkills();
  for (int i(0); i < skills.size(); i++)
  {
    addSkill(skills[i]);
  }
}

/**
 * @brief Task::addSkill
 * @param skill
 */
void Task::addSkill(Skill* skill)
{
  for (int i(0); i < skills_.size(); i++)
  {
    if (*skill == *skills_[i])
    {
      return;
    }
  }
  skills_.push_back(skill);
}

/**
 * @brief Task::removeSkill
 * @param skill
 */
void Task::removeSkill(const Skill& skill)
{
  for (int i(0); i < skills_.size(); i++)
  {
    if (skill == *skills_[i])
    {
      skills_.erase(skills_.begin() + i);
      return;
    }
  }
}

/**
 * @brief Task::setUser
 * @param user
 */
void Task::setUser(agents::User* user)
{
  if (user_)
  {
    delete user_;
    user_ = NULL;
  }
  user_ = user;
}

/**
 * @brief Task::setSender
 * @param sender
 */
void Task::setSender(agents::Person* sender)
{
  if (sender_)
  {
    delete sender_;
    sender_ = NULL;
  }
  sender_ = sender;
}

/**
 * @brief Task::setReceiver
 * @param receiver
 */
void Task::setReceiver(agents::Person* receiver)
{
  if (receiver_)
  {
    delete receiver_;
    receiver_ = NULL;
  }
  receiver_ = receiver;
}

/**
 * @brief Task::setPriority
 * @param priority
 */
void Task::setPriority(priorities::TaskPriorityEnum priority)
{
  priority_ = priority;
}

/**
 * @brief Task::setDeadline
 * @param deadline
 */
void Task::setDeadline(const ros::Time& deadline) { deadline_ = deadline; }

/**
 * @brief Task::setDeadline
 * @param duration
 */
void Task::setDeadline(const ros::Duration& duration)
{
  deadline_ = ros::Time::now() + duration;
}

/**
 * @brief Task::isExpired
 * @return
 */
bool Task::isExpired() const
{
  return deadline_.toSec() != 0.0 && deadline_ < ros::Time::now();
}

/**
 * @brief Task::isInvolved
 * @param person
 * @return
 */
bool Task::isInvolved(const agents::Person& person) const
{
  return person == *user_ || person == *receiver_ || person == *sender_;
}

/**
 * @brief Task::to_msg
 * @return
 */
mas_msgs::Task Task::to_msg() const
{
  mas_msgs::Task task_msg;
  task_msg.id = id_;
  task_msg.name = name_;
  task_msg.description = description_;
  for (int i(0); i < skills_.size(); i++)
  {
    if (skills_[i])
    {
      task_msg.skills.push_back(skills_[i]->to_msg());
    }
  }
  if (user_)
  {
    task_msg.user = user_->to_msg();
  }
  if (sender_)
  {
    task_msg.sender = sender_->to_msg();
  }
  if (receiver_)
  {
    task_msg.receiver = receiver_->to_msg();
  }
  task_msg.priority = TaskPriorities::toCode(priority_);
  task_msg.deadline = deadline_;
  return task_msg;
}

/**
 * @brief Task::str
 * @return
 */
std::string Task::str() const
{
  std::stringstream skills_ss;
  if (!skills_.empty())
  {
    if (skills_[0])
    {
      skills_ss << "0 " << skills_[0]->str();
    }
    for (int i(1); i < skills_.size(); i++)
    {
      if (skills_[i])
      {
        skills_ss << ", " << i << " " << skills_[i]->str();
      }
    }
  }
  return "task: {name: " + name_ +
         (!description_.empty() ? ", description: " + description_ : "") +
         (!skills_ss.str().empty() ? ", skills: {" + skills_ss.str() + "}" : "") +
         (user_ ? ", user: " + user_->str() : "") +
         (sender_ ? ", sender: " + sender_->str() : "") +
         (receiver_ ? ", receiver: " + receiver_->str() : "") + ", priority: " +
         TaskPriorities::str(priority_) +
         (!deadline_.isZero()
              ? ", deadline: " + utilities::TimeManipulator::str(deadline_)
              : "") +
         "}";
}

/**
 * @brief Task::c_str
 * @return
 */
const char* Task::c_str() const { return str().c_str(); }

/**
 * @brief Task::compareTo
 * @param task
 * @return
 */
int Task::compareTo(const Task& task) const
{
  return 10 * TaskPriorities::compare(priority_, task.priority_) +
         (user_ ? 2 * user_->compareTo(*task.user_) : 0) +
         (sender_ ? sender_->compareTo(*task.sender_) : 0) +
         (receiver_ ? receiver_->compareTo(*task.receiver_) : 0);
}

/**
 * @brief Task::operator =
 * @param task
 */
void Task::operator=(const Task& task)
{
  id_ = task.id_;
  name_ = task.name_;
  description_ = task.description_;
  setSkills(task.skills_);
  setUser(task.user_);
  setSender(task.sender_);
  setReceiver(task.receiver_);
  priority_ = task.priority_;
  deadline_ = task.deadline_;
}

/**
 * @brief Task::operator ==
 * @param task
 * @return
 */
bool Task::operator==(const Task& task) const { return id_ == task.id_; }

/**
 * @brief Task::operator !=
 * @param task
 * @return
 */
bool Task::operator!=(const Task& task) const { return !operator==(task); }
}
}
