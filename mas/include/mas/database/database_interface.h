/**
 *  This header file defines the DatabaseInterface class.
 *
 *  Version: 1.4.0
 *  Created on: 21/04/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _SYSTEM_DATABASE_INTERFACE_H_
#define _SYSTEM_DATABASE_INTERFACE_H_

#include "mas/tasks/allocation.h"

namespace mas
{
namespace database
{
class DatabaseInterface
{
public:
  DatabaseInterface();
  virtual ~DatabaseInterface();
  bool isComputerRegistered(std::string hostname) const;
  bool isPersonRegistered(std::string name) const;
  bool isRobotRegistered(std::string hostname) const;
  bool isTaskRegistered(std::string name) const;
  bool isUserRegistered(std::string name) const;
  std::string getUserLoginName(std::string name) const;
  agents::Computer getComputer(int id) const;
  agents::Person getPerson(int id) const;
  agents::Robot getRobot(int id) const;
  tasks::Task getTask(int id) const;
  agents::User getUser(int id) const;
  agents::Computer getComputer(std::string hostname) const;
  agents::Person getPerson(std::string name) const;
  agents::Robot getRobot(std::string hostname) const;
  tasks::Task getTask(std::string name) const;
  agents::User getUser(std::string login_name) const;
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

#endif /* _SYSTEM_DATABASE_INTERFACE_H_ */
