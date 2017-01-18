/**
 *  This source file implements the Robot Capability Resource class.
 *
 *  Version: 1.4.0
 *  Created on: 16/08/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrs/robots/resource.h"

namespace mrs
{
namespace robots
{

/**
 * @brief Resource::Resource
 * @param name
 */
Resource::Resource(std::string name) : id_(0), name_(name), description_("") {}

/**
 * @brief Resource::Resource
 * @param id
 * @param name
 * @param description
 */
Resource::Resource(int id, std::string name, std::string description)
    : id_(id), name_(name), description_(description)
{
}

/**
 * @brief Resource::Resource
 * @param resource
 */
Resource::Resource(const Resource& resource)
    : id_(resource.id_), name_(resource.name_),
      description_(resource.description_)
{
}

/**
 * @brief Resource::Resource
 * @param resource_msg
 */
Resource::Resource(const mrs_msgs::Resource::ConstPtr& resource_msg)
    : id_(resource_msg->id), name_(resource_msg->name),
      description_(resource_msg->description)
{
}

/**
 * @brief Resource::Resource
 * @param resource_msg
 */
Resource::Resource(const mrs_msgs::Resource& resource_msg)
    : id_(resource_msg.id), name_(resource_msg.name),
      description_(resource_msg.description)
{
}

/**
 * @brief Resource::~Resource
 */
Resource::~Resource() {}

/**
 * @brief Resource::getId
 * @return
 */
int Resource::getId() const { return id_; }

/**
 * @brief Resource::getName
 * @return
 */
std::string Resource::getName() const { return name_; }

/**
 * @brief Resource::getDescription
 * @return
 */
std::string Resource::getDescription() const { return description_; }

/**
 * @brief Resource::setDescription
 * @param description
 */
void Resource::setDescription(std::string description)
{
  description_ = description;
}

/**
 * @brief Resource::to_msg
 * @return
 */
mrs_msgs::Resource Resource::to_msg() const
{
  mrs_msgs::Resource resource_msg;
  resource_msg.id = id_;
  resource_msg.name = name_;
  resource_msg.description = description_;
  return resource_msg;
}

/**
 * @brief Resource::clone
 * @return
 */
Resource* Resource::clone() const { return new Resource(*this); }

/**
 * @brief Resource::str
 * @return
 */
std::string Resource::str() const
{
  return "resource: {name: " + name_ + ", description: " + description_ + "}";
}

/**
 * @brief Resource::c_str
 * @return
 */
const char* Resource::c_str() const { return str().c_str(); }

/**
 * @brief Resource::operator ==
 * @param resource
 * @return
 */
bool Resource::operator==(const Resource& resource) const
{
  return name_ == resource.name_;
}

/**
 * @brief Resource::operator !=
 * @param resource
 * @return
 */
bool Resource::operator!=(const Resource& resource) const
{
  return !operator==(resource);
}

/**
 * @brief Resource::operator =
 * @param resource
 */
void Resource::operator=(const Resource& resource)
{
  id_ = resource.id_;
  name_ = resource.name_;
  description_ = resource.description_;
}
}
}
