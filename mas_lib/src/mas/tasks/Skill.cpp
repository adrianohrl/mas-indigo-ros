/**
 *  Skill.cpp
 *
 *  Version: 1.2.4
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/Skill.h"

namespace mas
{
	namespace tasks
	{

		/**
		 *
		 */
		Skill::Skill(std::string resource_name, SkillLevelEnum level) : resource_(Resource(resource_name))
		{
		  id_ = 0;
		  level_ = level;
		}

		/**
		 *
		 */
		Skill::Skill(int id, Resource resource, SkillLevelEnum level) : resource_(resource)
		{
		  id_ = id;
		  level_ = level;
		}

		/**
		 *
		 */
		Skill::Skill(const mas_msgs::Skill::ConstPtr& skill_msg) : resource_(skill_msg->resource)
		{	
		  id_ = skill_msg->id;
			level_ = SkillLevels::toEnumerated(skill_msg->level);
		}

		/**
		 *
		 */
		Skill::Skill(mas_msgs::Skill skill_msg) : resource_(skill_msg.resource)
		{	
		  id_ = skill_msg.id;
			level_ = SkillLevels::toEnumerated(skill_msg.level);
		}

		/**
		 *
		 */
		Skill::~Skill() 
		{
		}

		/**
		 *
		 */
		int Skill::getId()
		{
		  return id_;
		}

		/**
		 *
		 */
		SkillLevelEnum Skill::getLevel()
		{
		  return level_;
		}

		/**
		 *
		 */
		Resource Skill::getResource() 
		{
			return resource_;
		}

		/**
		 * TESTAR
		 */
		bool Skill::isSufficient(SkillLevelEnum desired_level)
		{
		  return SkillLevels::isSufficient(level_, desired_level);
		}

		/**
		 * TESTAR
		 */
		bool Skill::isSufficient(Skill desired_skill)
		{
		  return equals(desired_skill) && isSufficient(desired_skill.level_);
		}

		/**
		 *
		 */
		void Skill::setId(int id)
		{
		  id_ = id;
		}

		/**
		 *
		 */
		void Skill::setLevel(SkillLevelEnum level)
		{
		  level_ = level;
		}

		/**
		 *
		 */
		mas_msgs::Skill Skill::toMsg()
		{
		  mas_msgs::Skill skill_msg;
		  skill_msg.id = id_;
		  skill_msg.resource = resource_.toMsg();
		  skill_msg.level = SkillLevels::toCode(level_);
		  return skill_msg;
		}

		/**
		 *
		 */
		std::string Skill::toString()
		{
			return "skill: {" + resource_.toString() +
					", level: " + SkillLevels::toString(level_) +
					"}";
		}

		/**
		 *
		 */
		int Skill::compareTo(Skill skill) 
		{
		    return level_ - skill.level_;
		}

		/**
		 *
		 */
		bool Skill::equals(Skill skill)
		{
			return operator==(skill);
		}

		/**
		 *
		 */
		bool Skill::operator==(const Skill& skill)
		{
			return resource_ == skill.resource_;
		}

		/**
		 *
		 */
		bool Skill::operator!=(const Skill& skill) 
		{
			return !operator==(skill);
		}

		/**
		 *
		 */
		void Skill::operator=(const Skill &skill)
		{ 
		  id_ = skill.id_;
			resource_ = skill.resource_;
			level_ = skill.level_;
		}
		
	}
}
