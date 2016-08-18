/**
 *  Resource.h
 *
 *  Version: 1.2.2
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASKS_RESOURCE_H_
#define TASKS_RESOURCE_H_

#include <string>
#include <mas_msgs/Resource.h>

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace tasks
			{
				class Resource 
				{
        public:
          Resource(std::string name);
          Resource(int id, std::string name, std::string description);
					Resource(const mas_msgs::Resource::ConstPtr& resource_msg);
					Resource(mas_msgs::Resource resource_msg);		
					~Resource();

					int getId();
					std::string getName();
					std::string getDescription();
					void setDescription(std::string description);
					mas_msgs::Resource toMsg();
          std::string toString();
					bool equals(Resource resource);
					bool operator==(const Resource& resource);
					bool operator!=(const Resource& resource);
					void operator=(const Resource& resource);

				private:
					int id_;
					std::string name_;
					std::string description_;	

				};
			}
		}
	}
}		
					
#endif /* TASKS_RESOURCE_H_ */
