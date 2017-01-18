/**
 *  This source file implements the Robot Capability class.
 *
 *  Version: 1.4.0
 *  Created on: 16/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrs/robots/capability.h"

namespace mrs
{
namespace robots
{

/**
 * @brief Capability::Capability
 * @param id
 * @param resource
 * @param lower_level
 * @param upper_level
 */
Capability::Capability(int id, const Resource& resource, Level lower_level,
                       Level upper_level)
    : id_(id), lower_level_(lower_level), upper_level_(upper_level),
      resource_(new Resource(resource))
{
}

/**
 * @brief Capability::Capability
 * @param capability
 */
Capability::Capability(const Capability& capability)
    : id_(capability.id_), lower_level_(capability.lower_level_),
      upper_level_(capability.upper_level_),
      resource_(new Resource(*capability.resource_))
{
}

/**
 * @brief Capability::Capability
 * @param capability_msg
 */
Capability::Capability(const mrs_msgs::Capability::ConstPtr& capability_msg)
    : id_(capability_msg->id),
      lower_level_(LevelConverter::toEnumerated(capability_msg->lower_level)),
      upper_level_(LevelConverter::toEnumerated(capability_msg->upper_level)),
      resource_(new Resource(capability_msg->resource))
{
}

/**
 * @brief Capability::Capability
 * @param capability_msg
 */
Capability::Capability(const mrs_msgs::Capability& capability_msg)
    : id_(capability_msg.id),
      lower_level_(LevelConverter::toEnumerated(capability_msg.lower_level)),
      upper_level_(LevelConverter::toEnumerated(capability_msg.upper_level)),
      resource_(new Resource(capability_msg.resource))
{
}

/**
 * @brief Capability::~Capability
 */
Capability::~Capability()
{
  if (resource_)
  {
    delete resource_;
    resource_ = NULL;
  }
}

/**
 * @brief Capability::getId
 * @return
 */
int Capability::getId() const { return id_; }

/**
 * @brief Capability::getLowerLevel
 * @return
 */
Level Capability::getLowerLevel() const { return lower_level_; }

/**
 * @brief Capability::getUpperLevel
 * @return
 */
Level Capability::getUpperLevel() const { return upper_level_; }

/**
 * @brief Capability::getResource
 * @return
 */
Resource* Capability::getResource() const { return resource_; }

/**
 * TESTAR
 */
bool Capability::isSufficient(Level level) const
{
  return false; // return LevelConverter::isSufficient(level_, desired_level);
}

/**
 * TESTAR
 */
bool Capability::isSufficient(const Capability& capability) const
{
  return false; // operator ==(desired_skill) &&
                // isSufficient(desired_skill.level_);
}

/**
 * @brief Capability::setId
 * @param id
 */
void Capability::setId(int id) { id_ = id; }

/**
 * @brief Capability::setLowerLevel
 * @param lower_level
 */
void Capability::setLowerLevel(Level lower_level)
{
  lower_level_ = lower_level;
}

/**
 * @brief Capability::setUpperLevel
 * @param upper_level
 */
void Capability::setUpperLevel(Level upper_level)
{
  upper_level_ = upper_level;
}

/**
 * @brief Capability::to_msg
 * @return
 */
mrs_msgs::Capability Capability::to_msg() const
{
  mrs_msgs::Capability skill_msg;
  skill_msg.id = id_;
  skill_msg.resource = resource_->to_msg();
  skill_msg.lower_level = lower_level_;
  skill_msg.upper_level = upper_level_;
  return skill_msg;
}

/**
 * @brief Capability::clone
 * @return
 */
Capability* Capability::clone() const { return new Capability(*this); }

/**
 * @brief Capability::str
 * @return
 */
std::string Capability::str() const
{
  return "skill: {" + resource_->str() + ", lower level: " +
         LevelConverter::toString(lower_level_) + ", upper level: " +
         LevelConverter::toString(upper_level_) + "}";
}

/**
 * @brief Capability::operator =
 * @param capability
 */
void Capability::operator=(const Capability& capability)
{
  id_ = capability.id_;
  if (capability.resource_)
  {
    delete resource_;
    resource_ = NULL;
  }
  resource_ = capability.resource_;
  lower_level_ = capability.lower_level_;
  upper_level_ = capability.upper_level_;
}

/**
 * @brief Capability::operator ==
 * @param capability
 * @return
 */
bool Capability::operator==(const Capability& capability) const
{
  return *resource_ == *capability.resource_;
}

/**
 * @brief Capability::operator !=
 * @param capability
 * @return
 */
bool Capability::operator!=(const Capability& capability) const
{
  return !operator==(capability);
}
}
}
