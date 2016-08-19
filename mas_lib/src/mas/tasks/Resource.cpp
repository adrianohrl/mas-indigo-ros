/**
 *  Resource.cpp
 *
 *  Version: 1.2.4
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/Resource.h"

namespace mas
{
	namespace tasks
	{

		/**
		 *
		 */
		Resource::Resource(std::string name)
		{
		  id_ = 0;
		  name_ = name;
		  description_ = "";
		}

		/**
		 *
		 */
		Resource::Resource(int id, std::string name, std::string description)
		{
		  id_ = id;
		  name_ = name;
		  description_ = description;
		}

		/**
		 *
		 */
		Resource::Resource(const mas_msgs::Resource::ConstPtr& resource_msg) 
		{
			id_ = resource_msg->id;	
			name_ = resource_msg->name;
			description_ = resource_msg->description;	
		}

		/**
		 *
		 */
		Resource::Resource(mas_msgs::Resource resource_msg) 
		{
			id_ = resource_msg.id;	
			name_ = resource_msg.name;
			description_ = resource_msg.description;	
		}

		/**
		 *
		 */
		Resource::~Resource() 
		{
		}

		/**
		 *
		 */
		int Resource::getId() 
		{
			return id_;
		}

		/**
		 *
		 */
		std::string Resource::getName() 
		{
			return name_;
		}

		/**
		 *
		 */
		std::string Resource::getDescription() 
		{
			return description_;
		}

		/**
		 *
		 */
		void Resource::setDescription(std::string description) 
		{
			description_ = description;
		}

		/**
		 *
		 */
		mas_msgs::Resource Resource::toMsg()
		{
		  mas_msgs::Resource resource_msg;
		  resource_msg.id = id_;
		  resource_msg.name = name_;
		  resource_msg.description = description_;
		  return resource_msg;
		}

		/**
		 *
		 */
		std::string Resource::toString()
		{
			return "resource: {name: " + name_ +
					", description: " + description_ +
					"}";
		}

		/**
		 *
		 */
		bool Resource::equals(Resource resource) 
		{
			return operator==(resource);
		}

		/**
		 *
		 */
		bool Resource::operator==(const Resource& resource)
		{
			return name_ == resource.name_;
		}

		/**
		 *
		 */
		bool Resource::operator!=(const Resource& resource) 
		{
			return !operator==(resource);
		}

		/**
		 *
		 */
		void Resource::operator=(const Resource &resource)
		{ 
			id_ = resource.id_;
			name_ = resource.name_;
			description_ = resource.description_;
		}
		
	}
}
