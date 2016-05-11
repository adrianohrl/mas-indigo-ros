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

				public:
					DatabaseInterface();	
          ~DatabaseInterface();
          bool isComputerRegistered(std::string hostname);
          bool isPersonRegistered(std::string name);
          bool isRobotRegistered(std::string hostname);
          bool isUserRegistered(std::string name);
          std::string getUserLoginName(std::string name);
          unifei::expertinos::mrta_vc::agents::Computer getComputer(int id);
          unifei::expertinos::mrta_vc::agents::Person getPerson(int id);
          unifei::expertinos::mrta_vc::agents::Robot getRobot(int id);
          unifei::expertinos::mrta_vc::agents::User getUser(int id);
          unifei::expertinos::mrta_vc::agents::Computer getComputer(std::string hostname);
          unifei::expertinos::mrta_vc::agents::Person getPerson(std::string name);
          unifei::expertinos::mrta_vc::agents::Robot getRobot(std::string hostname);
          unifei::expertinos::mrta_vc::agents::User getUser(std::string login_name);

				};
			}
		}
	}
}		
					
#endif /* DATABASE_INTERFACE_H_ */
