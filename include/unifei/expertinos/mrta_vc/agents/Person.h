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

#include <string>
#include "unifei/expertinos/mrta_vc/agents/Agent.h"

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
					Person(int id, std::string name, double x = 0, double y = 0, double theta = 0);
					Person(int id, std::string name, geometry_msgs::Pose pose_msg);
					Person(const ::mrta_vc::Agent::ConstPtr& person_msg);
					Person(::mrta_vc::Agent person_msg);		
					~Person();

					std::string getName();
					::mrta_vc::Agent toMsg();
					bool equals(Person person);
					bool operator==(const Person& person);
					bool operator!=(const Person& person);
					void operator=(const Person& person);
					
				protected:
					int getType();

				private:
					std::string name_;

				};
			}
		}
	}
}		
					
#endif /* PERSON_H_ */
