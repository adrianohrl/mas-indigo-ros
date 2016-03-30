/**
 *  Skill.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: **
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "br/edu/unifei/expertinos/mrta_vc/tasks/Skill.h"

/**
 *
 */
br::edu::unifei::expertinos::mrta_vc::tasks::Skill::Skill(Resource resource, int level) : resource_(resource)
{	
	level_ = level;
}

/**
 *
 */
br::edu::unifei::expertinos::mrta_vc::tasks::Skill::Skill(const ::mrta_vc::Skill::ConstPtr& skill_msg) : resource_(skill_msg->resource)
{	
	level_ = skill_msg->level;
}

/**
 *
 */
br::edu::unifei::expertinos::mrta_vc::tasks::Skill::Skill(::mrta_vc::Skill skill_msg) : resource_(skill_msg.resource)
{	
	level_ = skill_msg.level;
}

/**
 *
 */
br::edu::unifei::expertinos::mrta_vc::tasks::Skill::~Skill() 
{
}

/**
 *
 */
int br::edu::unifei::expertinos::mrta_vc::tasks::Skill::getLevel() 
{
	return level_;
}

/**
 *
 */
br::edu::unifei::expertinos::mrta_vc::tasks::Resource br::edu::unifei::expertinos::mrta_vc::tasks::Skill::getResource() 
{
	return resource_;
}

/**
 *
 */
void br::edu::unifei::expertinos::mrta_vc::tasks::Skill::setLevel(int level) 
{
	level_ = level;
}

/**
 *
 */
bool br::edu::unifei::expertinos::mrta_vc::tasks::Skill::isSufficient(Skill skill) 
{
	return resource_.equals(skill.resource_) && level_ <= skill.level_;	
}

/**
 * implementar
 */
int br::edu::unifei::expertinos::mrta_vc::tasks::Skill::compareTo(Skill skill) 
{
	return 0;
}

/**
 *
 */
::mrta_vc::Skill br::edu::unifei::expertinos::mrta_vc::tasks::Skill::toMsg() 
{
	::mrta_vc::Skill skill_msg;
	skill_msg.resource = resource_.toMsg();
	skill_msg.level = level_;
	return skill_msg;
}

/**
 *
 */
bool br::edu::unifei::expertinos::mrta_vc::tasks::Skill::operator==(const Skill& skill)
{
	return resource_ == skill.resource_;
}

/**
 *
 */
bool br::edu::unifei::expertinos::mrta_vc::tasks::Skill::operator!=(const Skill& skill) 
{
	return resource_ != skill.resource_;
}
