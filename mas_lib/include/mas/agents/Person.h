/**
 *  Person.h
 *
 *  Version: 1.2.4
 *  Created on: 05/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef AGENTS_PERSON_H_
#define AGENTS_PERSON_H_

#include <mas_msgs/Person.h>
#include "mas/agents/Agent.h"
#include "mas/agents/HierarchyLevels.h"

namespace mas 
{
	namespace agents
	{
		class Person : public Agent
		{

		public:
			Person();
			Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, double x = 0.0, double y = 0.0, double theta = 0.0);
			Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, geometry_msgs::Pose pose_msg);
			Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, mas::places::Location location);
			Person(const mas_msgs::Agent::ConstPtr& person_msg);
			Person(mas_msgs::Agent person_msg);		
			virtual ~Person();

			std::string getName();
			HierarchyLevelEnum getHierarchyLevel();
			mas_msgs::Agent toMsg();
			std::string toString();
			int compareTo(Person person);
			bool operator==(const Person& person);
			void operator=(const Person& person);
			
		protected:					
			int getType();
			void setName(std::string name);
			void setHierarchyLevel(HierarchyLevelEnum hierarchy_level);

		private:
			std::string name_;
			HierarchyLevelEnum hierarchy_level_;
			
			std::string getClassName();

		};
	}
}		
					
#endif /* AGENTS_PERSON_H_ */
