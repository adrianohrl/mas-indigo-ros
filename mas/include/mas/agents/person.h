/**
 *  This header file defines the Person class.
 *
 *  Version: 1.4.0
 *  Created on: 05/04/2016
 *  Modified on: 03/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _AGENTS_PERSON_H_
#define _AGENTS_PERSON_H_

#include <mas_msgs/Person.h>
#include "mas/agents/agent.h"
#include "mas/agents/hierarchy_levels.h"

namespace mas
{
namespace agents
{
class Person : public Agent
{

public:
  Person();
  Person(int id, std::string name, HierarchyLevelEnum hierarchy_level,
         double x = 0.0, double y = 0.0, double theta = 0.0);
  Person(int id, std::string name, HierarchyLevelEnum hierarchy_level,
         const geometry_msgs::Pose& pose_msg);
  Person(int id, std::string name, HierarchyLevelEnum hierarchy_level,
         const mas::places::Location& location);
  Person(const Person& person);
  Person(const mas_msgs::Agent::ConstPtr& person_msg);
  Person(const mas_msgs::Agent& person_msg);
  virtual ~Person();

  std::string getName() const;
  HierarchyLevelEnum getHierarchyLevel() const;
  virtual mas_msgs::Agent to_msg() const;
  virtual std::string str() const;
  virtual void operator=(const Person& person);
  virtual bool operator==(const Person& person) const;
  virtual bool operator==(const mas_msgs::Agent& msg) const;
  virtual bool operator==(const mas_msgs::Agent::ConstPtr& msg) const;
  int compareTo(Person person) const;

protected:
  virtual int getType() const;
  void setName(std::string name);
  void setHierarchyLevel(HierarchyLevelEnum hierarchy_level);

private:
  std::string name_;
  HierarchyLevelEnum hierarchy_level_;

  virtual std::string getClassName() const;
};
}
}

#endif /* _AGENTS_PERSON_H_ */
