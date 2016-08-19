/**
 *  DatabaseInterface.h
 *
 *  Version: 1.2.4
 *  Created on: 21/04/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_DATABASE_INTERFACE_H_
#define SYSTEM_DATABASE_INTERFACE_H_

#include "mas/tasks/Allocation.h"

namespace mas 
{
	namespace database
	{
		class DatabaseInterface 
		{

		protected:
			DatabaseInterface();	
			virtual ~DatabaseInterface();

			bool isComputerRegistered(std::string hostname);
			bool isPersonRegistered(std::string name);
			bool isRobotRegistered(std::string hostname);
			bool isTaskRegistered(std::string name);
			bool isUserRegistered(std::string name);
			std::string getUserLoginName(std::string name);
			agents::Computer getComputer(int id);
			agents::Person getPerson(int id);
			agents::Robot getRobot(int id);
			tasks::Task getTask(int id);
			agents::User getUser(int id);
			agents::Computer getComputer(std::string hostname);
			agents::Person getPerson(std::string name);
			agents::Robot getRobot(std::string hostname);
			tasks::Task getTask(std::string name);
			agents::User getUser(std::string login_name);
			int generateNewAgentId();
			int generateNewAllocationId();
			int generateNewPlaceId();
			int generateNewResourceId();
			int generateNewSkillId();
			int generateNewTaskId();

		private:
			int agents_counter_;
			int allocations_counter_;
			int places_counter_;
			int resources_counter_;
			int skills_counter_;
			int tasks_counter_;

		};
	}
}		
					
#endif /* SYSTEM_DATABASE_INTERFACE_H_ */
