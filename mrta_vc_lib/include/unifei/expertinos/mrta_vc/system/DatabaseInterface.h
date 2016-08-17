/**
 *  DatabaseInterface.h
 *
 *  Version: 1.0.0.0
 *  Created on: 21/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Christiano Henrique Rezende (c.h.rezende@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef DATABASE_INTERFACE_H_
#define DATABASE_INTERFACE_H_

#include "unifei/expertinos/mrta_vc/tasks/Allocation.h"

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace system
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
          unifei::expertinos::mrta_vc::agents::Computer getComputer(int id);
          unifei::expertinos::mrta_vc::agents::Person getPerson(int id);
          unifei::expertinos::mrta_vc::agents::Robot getRobot(int id);
          unifei::expertinos::mrta_vc::tasks::Task getTask(int id);
          unifei::expertinos::mrta_vc::agents::User getUser(int id);
          unifei::expertinos::mrta_vc::agents::Computer getComputer(std::string hostname);
          unifei::expertinos::mrta_vc::agents::Person getPerson(std::string name);
          unifei::expertinos::mrta_vc::agents::Robot getRobot(std::string hostname);
          unifei::expertinos::mrta_vc::tasks::Task getTask(std::string name);
          unifei::expertinos::mrta_vc::agents::User getUser(std::string login_name);
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
	}
}		
					
#endif /* DATABASE_INTERFACE_H_ */
