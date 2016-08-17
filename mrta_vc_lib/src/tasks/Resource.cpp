/**
 *  Resource.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/Resource.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Resource::Resource(std::string name)
{
  id_ = 0;
  name_ = name;
  description_ = "";
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Resource::Resource(int id, std::string name, std::string description)
{
  id_ = id;
  name_ = name;
  description_ = description;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Resource::Resource(const ::mrta_vc::Resource::ConstPtr& resource_msg) 
{
	id_ = resource_msg->id;	
	name_ = resource_msg->name;
	description_ = resource_msg->description;	
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Resource::Resource(::mrta_vc::Resource resource_msg) 
{
	id_ = resource_msg.id;	
	name_ = resource_msg.name;
	description_ = resource_msg.description;	
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Resource::~Resource() 
{
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::Resource::getId() 
{
	return id_;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::tasks::Resource::getName() 
{
	return name_;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::tasks::Resource::getDescription() 
{
	return description_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Resource::setDescription(std::string description) 
{
	description_ = description;
}

/**
 *
 */
::mrta_vc::Resource unifei::expertinos::mrta_vc::tasks::Resource::toMsg()
{
  ::mrta_vc::Resource resource_msg;
  resource_msg.id = id_;
  resource_msg.name = name_;
  resource_msg.description = description_;
  return resource_msg;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::tasks::Resource::toString()
{
	return "resource: {name: " + name_ +
			", description: " + description_ +
			"}";
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Resource::equals(unifei::expertinos::mrta_vc::tasks::Resource resource) 
{
	return operator==(resource);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Resource::operator==(const unifei::expertinos::mrta_vc::tasks::Resource& resource)
{
	return name_ == resource.name_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Resource::operator!=(const unifei::expertinos::mrta_vc::tasks::Resource& resource) 
{
	return !operator==(resource);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Resource::operator=(const unifei::expertinos::mrta_vc::tasks::Resource &resource)
{ 
	id_ = resource.id_;
	name_ = resource.name_;
	description_ = resource.description_;
}
