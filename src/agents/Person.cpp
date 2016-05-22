/**
 *  Person.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/08/2015
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/agents/Person.h"

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person::Person() : unifei::expertinos::mrta_vc::agents::Agent()
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person::Person(int id, std::string name, unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level, double x, double y, double theta) : unifei::expertinos::mrta_vc::agents::Agent(id, x, y, theta)
{
	name_ = name;
	hierarchy_level_ = hierarchy_level;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person::Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, geometry_msgs::Pose pose_msg) : unifei::expertinos::mrta_vc::agents::Agent(id, pose_msg)
{
	name_ = name;
	hierarchy_level_ = hierarchy_level;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person::Person(int id, std::string name, HierarchyLevelEnum hierarchy_level, unifei::expertinos::mrta_vc::places::Location location) : unifei::expertinos::mrta_vc::agents::Agent(id, location)
{
	name_ = name;
	hierarchy_level_ = hierarchy_level;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person::Person(const ::mrta_vc::Agent::ConstPtr& person_msg) : unifei::expertinos::mrta_vc::agents::Agent(person_msg)
{
	name_ = person_msg->name;
	hierarchy_level_ = unifei::expertinos::mrta_vc::agents::HierarchyLevels::toEnumerated(person_msg->hierarchy_level);
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person::Person(::mrta_vc::Agent person_msg) : unifei::expertinos::mrta_vc::agents::Agent(person_msg)
{
	name_ = person_msg.name;
	hierarchy_level_ = unifei::expertinos::mrta_vc::agents::HierarchyLevels::toEnumerated(person_msg.hierarchy_level);
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person::~Person() 
{
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::Person::getName() 
{
	return name_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum unifei::expertinos::mrta_vc::agents::Person::getHierarchyLevel() 
{
	return hierarchy_level_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::agents::Person::getType() 
{
	return PERSON;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::Person::getClassName() 
{
  return "person";
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Person::setName(std::string name) 
{
	name_ = name;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Person::setHierarchyLevel(unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level) 
{
	hierarchy_level_ = hierarchy_level;
}

/**
 *
 */
::mrta_vc::Agent unifei::expertinos::mrta_vc::agents::Person::toMsg() 
{
	::mrta_vc::Agent person_msg = unifei::expertinos::mrta_vc::agents::Agent::toMsg();
	person_msg.name = name_;
	person_msg.hierarchy_level = unifei::expertinos::mrta_vc::agents::HierarchyLevels::toCode(hierarchy_level_);
	return person_msg;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::agents::Person::toString() 
{
  return unifei::expertinos::mrta_vc::agents::Agent::toString() + ", name: " + name_ + ", hierarchy level: " + unifei::expertinos::mrta_vc::agents::HierarchyLevels::toString(hierarchy_level_) + "}";
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Person::operator==(const unifei::expertinos::mrta_vc::agents::Person& person)
{
	return name_ == person.name_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Person::operator=(const unifei::expertinos::mrta_vc::agents::Person& person) 
{
	unifei::expertinos::mrta_vc::agents::Agent::operator=(person);
	name_ = person.name_;
	hierarchy_level_ = person.hierarchy_level_;
}
