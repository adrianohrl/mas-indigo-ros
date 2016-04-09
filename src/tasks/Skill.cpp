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
unifei::expertinos::mrta_vc::tasks::Skill::Skill(unifei::expertinos::mrta_vc::tasks::Resource resource, unifei::expertinos::mrta_vc::tasks::SkillLevelEnum level) : resource_(resource)
{	
	level_ = level;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Skill::Skill(const ::mrta_vc::Skill::ConstPtr& skill_msg) : resource_(skill_msg->resource)
{	
	level_ = unifei::expertinos::mrta_vc::tasks::SkillLevels::toEnumerated(skill_msg->level);
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Skill::Skill(::mrta_vc::Skill skill_msg) : resource_(skill_msg.resource)
{	
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
	skill_msg.resource = resource_.toMsg();
	skill_msg.level = unifei::expertinos::mrta_vc::tasks::SkillLevels::toCode(level_);
	return skill_msg;
}

/**
 * TESTAR
 */
bool unifei::expertinos::mrta_vc::tasks::Skill::isSufficient(unifei::expertinos::mrta_vc::tasks::Skill skill) 
{ 
	return equals(skill) && unifei::expertinos::mrta_vc::tasks::SkillLevels::toCode(level_) <= unifei::expertinos::mrta_vc::tasks::SkillLevels::toCode(skill.level_);	
}

/**
 * 
 */
bool unifei::expertinos::mrta_vc::tasks::Skill::equals(unifei::expertinos::mrta_vc::tasks::Skill skill) 
{ 
	return resource_.equals(skill.resource_);	
}

/**
 * IMPLEMENTAR
 */
int unifei::expertinos::mrta_vc::tasks::Skill::compareTo(unifei::expertinos::mrta_vc::tasks::Skill skill) 
{
	return 0;
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
	return resource_ != skill.resource_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Skill::operator=(const unifei::expertinos::mrta_vc::tasks::Skill &skill)
{ 
	resource_ = skill.resource_;
	level_ = skill.level_;
}
