/**
 *  This source file implements the Skill class.
 *
 *  Version: 1.4.0
 *  Created on: 04/08/2015
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/skill.h"

namespace mas
{
namespace tasks
{

/**
 * @brief Skill::Skill
 * @param resource_name
 * @param level
 */
Skill::Skill(std::string resource_name, SkillLevelEnum level)
    : id_(0), level_(level), resource_(new Resource(resource_name))
{
}

/**
 * @brief Skill::Skill
 * @param id
 * @param resource
 * @param level
 */
Skill::Skill(int id, const Resource& resource, SkillLevelEnum level)
    : id_(id), level_(level), resource_(new Resource(resource))
{
}

/**
 * @brief Skill::Skill
 * @param id
 * @param resource
 * @param level
 */
Skill::Skill(int id, Resource* resource, SkillLevelEnum level)
    : id_(id), level_(level), resource_(resource)
{
}

/**
 * @brief Skill::Skill copies the given Skill object.
 * @param skill
 */
Skill::Skill(const Skill& skill)
    : id_(skill.id_), level_(skill.level_),
      resource_(new Resource(*skill.resource_))
{
}

/**
 * @brief Skill::Skill
 * @param skill_msg
 */
Skill::Skill(const mas_msgs::Skill::ConstPtr& skill_msg)
    : id_(skill_msg->id), level_(SkillLevels::toEnumerated(skill_msg->level)),
      resource_(new Resource(skill_msg->resource))
{
}

/**
 * @brief Skill::Skill
 * @param skill_msg
 */
Skill::Skill(const mas_msgs::Skill& skill_msg)
    : id_(skill_msg.id), level_(SkillLevels::toEnumerated(skill_msg.level)),
      resource_(new Resource(skill_msg.resource))
{
}

/**
 * @brief Skill::~Skill
 */
Skill::~Skill()
{
  if (resource_)
  {
    delete resource_;
    resource_ = NULL;
  }
}

/**
 * @brief Skill::getId
 * @return
 */
int Skill::getId() const { return id_; }

/**
 * @brief Skill::getLevel
 * @return
 */
SkillLevelEnum Skill::getLevel() const { return level_; }

/**
 * @brief Skill::getResource
 * @return
 */
Resource* Skill::getResource() const { return resource_; }

/**
 * @brief Skill::isSufficient
 * @param desired_level
 * @return
 */
bool Skill::isSufficient(SkillLevelEnum desired_level) const
{
  return SkillLevels::isSufficient(level_, desired_level);
}

/**
 * @brief Skill::isSufficient
 * @param desired_skill
 * @return
 */
bool Skill::isSufficient(const Skill& desired_skill) const
{
  return operator==(desired_skill) && isSufficient(desired_skill.level_);
}

/**
 * @brief Skill::setId
 * @param id
 */
void Skill::setId(int id) { id_ = id; }

/**
 * @brief Skill::setLevel
 * @param level
 */
void Skill::setLevel(SkillLevelEnum level) { level_ = level; }

/**
 * @brief Skill::to_msg
 * @return
 */
mas_msgs::Skill Skill::to_msg() const
{
  mas_msgs::Skill skill_msg;
  skill_msg.id = id_;
  if (resource_)
  {
    skill_msg.resource = resource_->to_msg();
  }
  skill_msg.level = SkillLevels::toCode(level_);
  return skill_msg;
}

/**
 * @brief Skill::str
 * @return
 */
std::string Skill::str() const
{
  return "skill: {" + (resource_ ? resource_->str() : "") + ", level: " +
         SkillLevels::str(level_) + "}";
}

/**
 * @brief Skill::c_str
 * @return
 */
const char* Skill::c_str() const { return str().c_str(); }

/**
 * @brief Skill::operator =
 * @param skill
 */
void Skill::operator=(const Skill& skill)
{
  id_ = skill.id_;
  if (resource_)
  {
    delete resource_;
    resource_ = NULL;
  }
  resource_ = new Resource(*skill.resource_);
  level_ = skill.level_;
}

/**
 * @brief Skill::operator ==
 * @param skill
 * @return
 */
bool Skill::operator==(const Skill& skill) const
{
  return resource_ && skill.resource_ && *resource_ == *skill.resource_;
}

/**
 * @brief Skill::operator !=
 * @param skill
 * @return
 */
bool Skill::operator!=(const Skill& skill) const { return !operator==(skill); }

/**
 * @brief Skill::compareTo
 * @param skill
 * @return
 */
int Skill::compareTo(const Skill& skill) const { return level_ - skill.level_; }
}
}
