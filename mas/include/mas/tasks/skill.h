/**
 *  This header file defines the Skill class.
 *
 *  Version: 1.4.0
 *  Created on: 04/08/2015
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASKS_SKILL_H_
#define _TASKS_SKILL_H_

#include <mas_msgs/Skill.h>
#include "mas/tasks/resource.h"
#include "mas/tasks/skill_levels.h"

namespace mas
{
namespace tasks
{
class Skill
{
public:
  Skill(std::string resource_name,
        SkillLevelEnum level = SkillLevels::getDefault());
  Skill(int id, const Resource& resource,
        SkillLevelEnum level = SkillLevels::getDefault());
  Skill(int id, Resource* resource,
        SkillLevelEnum level = SkillLevels::getDefault());
  Skill(const Skill& skill);
  Skill(const mas_msgs::Skill::ConstPtr& skill_msg);
  Skill(const mas_msgs::Skill& skill_msg);
  ~Skill();

  int getId() const;
  SkillLevelEnum getLevel() const;
  Resource* getResource() const;
  bool isSufficient(SkillLevelEnum desired_level) const;
  bool isSufficient(const Skill& desired_skill) const;
  void setId(int id);
  void setLevel(SkillLevelEnum level);
  virtual mas_msgs::Skill to_msg() const;
  virtual std::string str() const;
  const char* c_str() const;
  void operator=(const Skill& skill);
  bool operator==(const Skill& skill) const;
  bool operator!=(const Skill& skill) const;
  int compareTo(const Skill& skill) const;

private:
  int id_;
  Resource* resource_;
  SkillLevelEnum level_;
};
}
}

#endif /* _TASKS_SKILL_H_ */
