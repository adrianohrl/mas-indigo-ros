/**
 *  This source file implements the Person class.
 *
 *  Version: 1.4.0
 *  Created on: 04/08/2015
 *  Modified on: 03/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/person.h"

namespace mas
{
namespace agents
{

/**
 * @brief Person::Person
 */
Person::Person() : Agent(), hierarchy_level_(levels::INTERN) {}

/**
 * @brief Person::Person
 * @param id
 * @param name
 * @param hierarchy_level
 * @param x
 * @param y
 * @param theta
 */
Person::Person(int id, std::string name, HierarchyLevelEnum hierarchy_level,
               double x, double y, double theta)
    : Agent(id, x, y, theta), name_(name), hierarchy_level_(hierarchy_level)
{
}

/**
 * @brief Person::Person
 * @param id
 * @param name
 * @param hierarchy_level
 * @param pose_msg
 */
Person::Person(int id, std::string name, HierarchyLevelEnum hierarchy_level,
               const geometry_msgs::Pose& pose_msg)
    : Agent(id, pose_msg), name_(name), hierarchy_level_(hierarchy_level)
{
}

/**
 * @brief Person::Person
 * @param id
 * @param name
 * @param hierarchy_level
 * @param location
 */
Person::Person(int id, std::string name, HierarchyLevelEnum hierarchy_level,
               const places::Location& location)
    : Agent(id, location), name_(name), hierarchy_level_(hierarchy_level)
{
}

/**
 * @brief Person::Person
 * @param person
 */
Person::Person(const Person& person)
    : Agent(person), name_(person.name_),
      hierarchy_level_(person.hierarchy_level_)
{
}

/**
 * @brief Person::Person
 * @param person_msg
 */
Person::Person(const mas_msgs::Agent::ConstPtr& person_msg)
    : Agent(person_msg), name_(person_msg->name),
      hierarchy_level_(
          HierarchyLevels::toEnumerated(person_msg->hierarchy_level))
{
}

/**
 * @brief Person::Person
 * @param person_msg
 */
Person::Person(const mas_msgs::Agent& person_msg)
    : Agent(person_msg), name_(person_msg.name),
      hierarchy_level_(
          HierarchyLevels::toEnumerated(person_msg.hierarchy_level))
{
}

/**
 * @brief Person::~Person
 */
Person::~Person() {}

/**
 * @brief Person::getName
 * @return
 */
std::string Person::getName() const { return name_; }

/**
 * @brief Person::getHierarchyLevel
 * @return
 */
HierarchyLevelEnum Person::getHierarchyLevel() const
{
  return hierarchy_level_;
}

/**
 * @brief Person::getType
 * @return
 */
int Person::getType() const { return PERSON; }

/**
 * @brief Person::getClassName
 * @return
 */
std::string Person::getClassName() const { return "person"; }

/**
 * @brief Person::setName
 * @param name
 */
void Person::setName(std::string name) { name_ = name; }

/**
 * @brief Person::setHierarchyLevel
 * @param hierarchy_level
 */
void Person::setHierarchyLevel(HierarchyLevelEnum hierarchy_level)
{
  hierarchy_level_ = hierarchy_level;
}

/**
 * @brief Person::to_msg
 * @return
 */
mas_msgs::Agent Person::to_msg() const
{
  mas_msgs::Agent person_msg(Agent::to_msg());
  person_msg.name = name_;
  person_msg.hierarchy_level = HierarchyLevels::toCode(hierarchy_level_);
  return person_msg;
}

/**
 * @brief Person::str
 * @return
 */
std::string Person::str() const
{
  return Agent::str() + ", name: " + name_ + ", hierarchy level: " +
         HierarchyLevels::str(hierarchy_level_) + "}";
}

/**
 * @brief Person::operator =
 * @param person
 */
void Person::operator=(const Person& person)
{
  Agent::operator=(person);
  name_ = person.name_;
  hierarchy_level_ = person.hierarchy_level_;
}

/**
 * @brief Person::operator ==
 * @param person
 * @return
 */
bool Person::operator==(const Person& person) const
{
  return name_ == person.name_;
}

/**
 * @brief Person::operator ==
 * @param msg
 * @return
 */
bool Person::operator==(const mas_msgs::Agent& msg) const
{
  return name_ == msg.name;
}

/**
 * @brief Person::operator ==
 * @param msg
 * @return
 */
bool Person::operator==(const mas_msgs::Agent::ConstPtr& msg) const
{
  return name_ == msg->name;
}

/**
 * @brief Person::compareTo
 * @param person
 * @return
 */
int Person::compareTo(Person person) const
{
  return HierarchyLevels::compare(hierarchy_level_, person.hierarchy_level_);
}
}
}
