/**
 *  Person.cpp
 *
 *  Version: 1.2.4
 *  Created on: 04/08/2015
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/Person.h"

namespace mas
{
	namespace agents
	{

		/**
		 *
		 */
		Person::Person() : Agent()
		{
			hierarchy_level_ = levels::INTERN;
		}

		/**
		 *
		 */
		Person::Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, double x, double y, double theta) : Agent(id, x, y, theta)
		{
			name_ = name;
			hierarchy_level_ = hierarchy_level;
		}

		/**
		 *
		 */
		Person::Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, geometry_msgs::Pose pose_msg) : Agent(id, pose_msg)
		{
			name_ = name;
			hierarchy_level_ = hierarchy_level;
		}

		/**
		 *
		 */
		Person::Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, places::Location location) : Agent(id, location)
		{
			name_ = name;
			hierarchy_level_ = hierarchy_level;
		}

		/**
		 *
		 */
		Person::Person(const mas_msgs::Agent::ConstPtr& person_msg) : Agent(person_msg)
		{
			name_ = person_msg->name;
			hierarchy_level_ = HierarchyLevels::toEnumerated(person_msg->hierarchy_level);
		}

		/**
		 *
		 */
		Person::Person(mas_msgs::Agent person_msg) : Agent(person_msg)
		{
			name_ = person_msg.name;
			hierarchy_level_ = HierarchyLevels::toEnumerated(person_msg.hierarchy_level);
		}

		/**
		 *
		 */
		Person::~Person() 
		{
		}

		/**
		 *
		 */
		std::string Person::getName() 
		{
			return name_;
		}

		/**
		 *
		 */
		HierarchyLevelEnum Person::getHierarchyLevel() 
		{
			return hierarchy_level_;
		}

		/**
		 *
		 */
		int Person::getType() 
		{
			return PERSON;
		}

		/**
		 *
		 */
		std::string Person::getClassName() 
		{
		  return "person";
		}

		/**
		 *
		 */
		void Person::setName(std::string name) 
		{
			name_ = name;
		}

		/**
		 *
		 */
		void Person::setHierarchyLevel(HierarchyLevelEnum hierarchy_level) 
		{
			hierarchy_level_ = hierarchy_level;
		}

		/**
		 *
		 */
		mas_msgs::Agent Person::toMsg() 
		{
			mas_msgs::Agent person_msg = Agent::toMsg();
			person_msg.name = name_;
			person_msg.hierarchy_level = HierarchyLevels::toCode(hierarchy_level_);
			return person_msg;
		}

		/**
		 *
		 */
		std::string Person::toString() 
		{
			return Agent::toString() +
					", name: " + name_ +
					", hierarchy level: " + HierarchyLevels::toString(hierarchy_level_) +
					"}";
		}

		/**
		 *
		 */
		int Person::compareTo(Person person)
		{
			return HierarchyLevels::compare(hierarchy_level_, person.hierarchy_level_);
		}

		/**
		 *
		 */
		bool Person::operator==(const Person& person)
		{
			return name_ == person.name_;
		}

		/**
		 *
		 */
		void Person::operator=(const Person& person) 
		{
			Agent::operator=(person);
			name_ = person.name_;
			hierarchy_level_ = person.hierarchy_level_;
		}
		
	}
}
