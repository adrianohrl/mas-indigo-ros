/**
 *  Skill.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/Skill.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Skill::Skill(std::string resource_name, unifei::expertinos::mrta_vc::tasks::SkillLevelEnum level) : resource_(unifei::expertinos::mrta_vc::tasks::Resource(resource_name))
{
  id_ = 0;
  level_ = level;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Skill::Skill(int id, unifei::expertinos::mrta_vc::tasks::Resource resource, unifei::expertinos::mrta_vc::tasks::SkillLevelEnum level) : resource_(resource)
{
  id_ = id;
  level_ = level;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Skill::Skill(const ::mrta_vc::Skill::ConstPtr& skill_msg) : resource_(skill_msg->resource)
{	
  id_ = skill_msg->id;
	level_ = unifei::expertinos::mrta_vc::tasks::SkillLevels::toEnumerated(skill_msg->level);
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Skill::Skill(::mrta_vc::Skill skill_msg) : resource_(skill_msg.resource)
{	
  id_ = skill_msg.id;
	level_ = unifei::expertinos::mrta_vc::tasks::SkillLevels::toEnumerated(skill_msg.level);
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Skill::~Skill() 
{
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::Skill::getId()
{
  return id_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::SkillLevelEnum unifei::expertinos::mrta_vc::tasks::Skill::getLevel()
{
  return level_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Resource unifei::expertinos::mrta_vc::tasks::Skill::getResource() 
{
	return resource_;
}

/**
 * TESTAR
 */
bool unifei::expertinos::mrta_vc::tasks::Skill::isSufficient(unifei::expertinos::mrta_vc::tasks::SkillLevelEnum desired_level)
{
  return unifei::expertinos::mrta_vc::tasks::SkillLevels::isSufficient(level_, desired_level);
}

/**
 * TESTAR
 */
bool unifei::expertinos::mrta_vc::tasks::Skill::isSufficient(unifei::expertinos::mrta_vc::tasks::Skill desired_skill)
{
  return equals(desired_skill) && isSufficient(desired_skill.level_);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Skill::setId(int id)
{
  id_ = id;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Skill::setLevel(unifei::expertinos::mrta_vc::tasks::SkillLevelEnum level)
{
  level_ = level;
}

/**
 *
 */
::mrta_vc::Skill unifei::expertinos::mrta_vc::tasks::Skill::toMsg()
{
  ::mrta_vc::Skill skill_msg;
  skill_msg.id = id_;
  skill_msg.resource = resource_.toMsg();
  skill_msg.level = unifei::expertinos::mrta_vc::tasks::SkillLevels::toCode(level_);
  return skill_msg;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::tasks::Skill::toString()
{
  return "skill: {" + resource_.toString() + ", level: " + unifei::expertinos::mrta_vc::tasks::SkillLevels::toString(level_) + "}";
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::Skill::compareTo(unifei::expertinos::mrta_vc::tasks::Skill skill) 
{
    return level_ - skill.level_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Skill::equals(unifei::expertinos::mrta_vc::tasks::Skill skill)
{
	return operator==(skill);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Skill::operator==(const unifei::expertinos::mrta_vc::tasks::Skill& skill)
{
	return resource_ == skill.resource_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Skill::operator!=(const unifei::expertinos::mrta_vc::tasks::Skill& skill) 
{
	return !operator==(skill);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Skill::operator=(const unifei::expertinos::mrta_vc::tasks::Skill &skill)
{ 
  id_ = skill.id_;
	resource_ = skill.resource_;
	level_ = skill.level_;
}
