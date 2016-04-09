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
unifei::expertinos::mrta_vc::agents::Person::Person(int id, std::string name, double x, double y, double theta) : Agent(id, x, y, theta)
{
	name_ = name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person::Person(int id, std::string name, geometry_msgs::Pose pose_msg) : Agent(id, pose_msg)
{
	name_ = name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person::Person(const ::mrta_vc::Agent::ConstPtr& person_msg) : Agent(person_msg)
{
	name_ = person_msg->name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person::Person(::mrta_vc::Agent person_msg) : Agent(person_msg)
{
	name_ = person_msg.name;
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
int unifei::expertinos::mrta_vc::agents::Person::getType() 
{
	return PERSON;
}

/**
 *
 */
::mrta_vc::Agent unifei::expertinos::mrta_vc::agents::Person::toMsg() 
{
	::mrta_vc::Agent person_msg = Agent::toMsg();
	person_msg.name = name_;
	return person_msg;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::Person::equals(unifei::expertinos::mrta_vc::agents::Person person) 
{
	return name_ == person.name_;
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
bool unifei::expertinos::mrta_vc::agents::Person::operator!=(const unifei::expertinos::mrta_vc::agents::Person& person) 
{
	return name_ != person.name_;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::agents::Person::operator=(const unifei::expertinos::mrta_vc::agents::Person& person) 
{
	unifei::expertinos::mrta_vc::agents::Agent::operator=(person);
	name_ = person.name_;
}
