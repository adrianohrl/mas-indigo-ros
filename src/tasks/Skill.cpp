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
unifei::expertinos::mrta_vc::tasks::Skill::Skill(Resource resource, int level) : resource_(resource)
{	
	level_ = level;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Skill::Skill(const ::mrta_vc::Skill::ConstPtr& skill_msg) : resource_(skill_msg->resource)
{	
	level_ = skill_msg->level;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Skill::Skill(::mrta_vc::Skill skill_msg) : resource_(skill_msg.resource)
{	
	level_ = skill_msg.level;
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
int unifei::expertinos::mrta_vc::tasks::Skill::getLevel() 
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
void unifei::expertinos::mrta_vc::tasks::Skill::setLevel(int level) 
{
	level_ = level;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Skill::isSufficient(Skill skill) 
{
	return resource_.equals(skill.resource_) && level_ <= skill.level_;	
}

/**
 * implementar
 */
int unifei::expertinos::mrta_vc::tasks::Skill::compareTo(Skill skill) 
{
	return 0;
}

/**
 *
 */
::mrta_vc::Skill unifei::expertinos::mrta_vc::tasks::Skill::toMsg() 
{
	::mrta_vc::Skill skill_msg;
	skill_msg.resource = resource_.toMsg();
	skill_msg.level = level_;
	return skill_msg;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Skill::operator==(const Skill& skill)
{
	return resource_ == skill.resource_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Skill::operator!=(const Skill& skill) 
{
	return resource_ != skill.resource_;
}
