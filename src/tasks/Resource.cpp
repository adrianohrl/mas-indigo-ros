/**
 *  Resource.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: **
 *  Author: Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "br/edu/unifei/expertinos/mrta_vc/tasks/Resource.h"

/**
 *
 */
br::edu::unifei::expertinos::mrta_vc::tasks::Resource::Resource(int id, std::string name, std::string description) 
{
	id_ = id;
	name_ = name;
	description_ = description;	
}

/**
 *
 */
br::edu::unifei::expertinos::mrta_vc::tasks::Resource::Resource(const ::mrta_vc::Resource::ConstPtr& resource_msg) 
{
	id_ = resource_msg->id;	
	name_ = resource_msg->name;
	description_ = resource_msg->description;	
}

/**
 *
 */
br::edu::unifei::expertinos::mrta_vc::tasks::Resource::Resource(::mrta_vc::Resource resource_msg) 
{
	id_ = resource_msg.id;	
	name_ = resource_msg.name;
	description_ = resource_msg.description;	
}

/**
 *
 */
br::edu::unifei::expertinos::mrta_vc::tasks::Resource::~Resource() 
{
}

/**
 *
 */
int br::edu::unifei::expertinos::mrta_vc::tasks::Resource::getId() 
{
	return id_;
}

/**
 *
 */
std::string br::edu::unifei::expertinos::mrta_vc::tasks::Resource::getName() 
{
	return name_;
}

/**
 *
 */
std::string br::edu::unifei::expertinos::mrta_vc::tasks::Resource::getDescription() 
{
	return description_;
}

/**
 *
 */
void br::edu::unifei::expertinos::mrta_vc::tasks::Resource::setDescription(std::string description) 
{
	description_ = description;
}

/**
 *
 */
bool br::edu::unifei::expertinos::mrta_vc::tasks::Resource::equals(Resource resource) 
{
	return id_ == resource.id_;
}

/**
 *
 */
::mrta_vc::Resource br::edu::unifei::expertinos::mrta_vc::tasks::Resource::toMsg() 
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
bool br::edu::unifei::expertinos::mrta_vc::tasks::Resource::operator==(const Resource& resource)
{
	return id_ == resource.id_;
}

/**
 *
 */
bool br::edu::unifei::expertinos::mrta_vc::tasks::Resource::operator!=(const Resource& resource) 
{
	return id_ != resource.id_;
}
