/**
 *  Person.h
 *
 *  Version: 1.0.0.0
 *  Created on: 05/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PERSON_H_
#define PERSON_H_

#include "unifei/expertinos/mrta_vc/agents/Agent.h"
#include "unifei/expertinos/mrta_vc/agents/HierarchyLevels.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace agents
			{
				class Person : public Agent
				{

				public:
          Person();
					Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, double x = 0, double y = 0, double theta = 0);
					Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, geometry_msgs::Pose pose_msg);
					Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, unifei::expertinos::mrta_vc::places::Location location);
					Person(const ::mrta_vc::Agent::ConstPtr& person_msg);
					Person(::mrta_vc::Agent person_msg);		
					~Person();

					std::string getName();
					HierarchyLevelEnum getHierarchyLevel();
					::mrta_vc::Agent toMsg();
					std::string toString();
					bool equals(Person person);
					bool operator==(const Person& person);
					bool operator!=(const Person& person);
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
	}
}		
					
#endif /* PERSON_H_ */
