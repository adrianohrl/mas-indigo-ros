/**
 *  This source file implements the DatabaseInterface class.
 *
 *  Version: 1.4.0
 *  Created on: 21/04/2016
 *  Modified on: 14/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/database/database_interface.h"

namespace mas
{
namespace database
{

/**
 * @brief DatabaseInterface::DatabaseInterface
 */
DatabaseInterface::DatabaseInterface()
    : agents_counter_(0), allocations_counter_(0), places_counter_(0),
      resources_counter_(0), skills_counter_(0), tasks_counter_(0)
{
}

/**
 * @brief DatabaseInterface::~DatabaseInterface
 */
DatabaseInterface::~DatabaseInterface() {}

/**
 * @brief DatabaseInterface::isComputerRegistered
 * @param hostname
 * @return
 */
bool DatabaseInterface::isComputerRegistered(std::string hostname) const
{
  return hostname == "adrianohrl-pc" || hostname == "luis-pc" ||
         hostname == "heverton-pc" || hostname == "audeliano-pc";
}

/**
 * @brief DatabaseInterface::isPersonRegistered
 * @param name
 * @return
 */
bool DatabaseInterface::isPersonRegistered(std::string name) const
{
  return name == "Adriano Henrique Rossette Leite" ||
         name == "Luís Victor Pessiqueli Bonin" || name == "Héverton Soares" ||
         name == "Audeliano Li";
}

/**
 * @brief DatabaseInterface::isRobotRegistered
 * @param hostname
 * @return
 */
bool DatabaseInterface::isRobotRegistered(std::string hostname) const
{
  return hostname == "robotino1" || hostname == "robotino2" ||
         hostname == "robotino3" || hostname == "p3_dx";
}

/**
 * @brief DatabaseInterface::isTaskRegistered
 * @param name
 * @return
 */
bool DatabaseInterface::isTaskRegistered(std::string name) const
{
  return name == "bring water" || name == "take water" ||
         name == "send water" || name == "bring document" ||
         name == "take document" || name == "send document" ||
         name == "bring meal" || name == "take meal" || name == "send meal" ||
         name == "bring coffee" || name == "take coffee" ||
         name == "send coffee" || name == "bring gallon" ||
         name == "take gallon" || name == "send gallon";
}

/**
 * @brief DatabaseInterface::isUserRegistered
 * @param name
 * @return
 */
bool DatabaseInterface::isUserRegistered(std::string name) const
{
  return name == "Adriano Henrique Rossette Leite" ||
         name == "Luís Victor Pessiqueli Bonin" || name == "Héverton Soares" ||
         name == "Audeliano Li";
}

/**
 * @brief DatabaseInterface::getUserLoginName
 * @param name
 * @return
 */
std::string DatabaseInterface::getUserLoginName(std::string name) const
{
  std::string login_name("");
  if (name == "Adriano Henrique Rossette Leite")
  {
    login_name = "adrianohrl";
  }
  else if (name == "Luís Victor Pessiqueli Bonin")
  {
    login_name = "luis";
  }
  else if (name == "Héverton Soares")
  {
    login_name = "heverton";
  }
  else if (name == "Audeliano Li")
  {
    login_name = "audeliano";
  }
  return login_name;
}

/**
 * @brief DatabaseInterface::getComputer
 * @param id
 * @return
 */
agents::Computer DatabaseInterface::getComputer(int id) const
{
  agents::Computer computer;
  switch (id)
  {
  case 201:
    computer = getComputer("adrianohrl-pc");
    break;
  case 203:
    computer = getComputer("luis-pc");
    break;
  case 204:
    computer = getComputer("heverton-pc");
    break;
  case 205:
    computer = getComputer("audeliano-pc");
    break;
  }
  return computer;
}

/**
 * @brief DatabaseInterface::getPerson
 * @param id
 * @return
 */
agents::Person DatabaseInterface::getPerson(int id) const
{
  agents::Person person;
  switch (id)
  {
  case 101:
    person = getPerson("Adriano Henrique Rossette Leite");
    break;
  case 103:
    person = getPerson("Luís Victor Pessiqueli Bonin");
    break;
  case 104:
    person = getPerson("Héverton Soares");
    break;
  case 105:
    person = getPerson("Audeliano Li");
    break;
  }
  return person;
}

/**
 * @brief DatabaseInterface::getRobot
 * @param id
 * @return
 */
agents::Robot DatabaseInterface::getRobot(int id) const
{
  agents::Robot robot;
  switch (id)
  {
  case 301:
    robot = getRobot("robotino1");
    break;
  case 302:
    robot = getRobot("robotino2");
    break;
  case 303:
    robot = getRobot("robotino3");
    break;
  case 304:
    robot = getRobot("p3_dx");
    break;
  }
  return robot;
}

/**
 * @brief DatabaseInterface::getTask
 * @param id
 * @return
 */
tasks::Task DatabaseInterface::getTask(int id) const
{

  tasks::Task task;
  switch (id)
  {
  case 401:
    task = getTask("bring water");
    break;
  case 402:
    task = getTask("take water");
    break;
  case 403:
    task = getTask("send water");
    break;
  case 404:
    task = getTask("bring document");
    break;
  case 405:
    task = getTask("take document");
    break;
  case 406:
    task = getTask("send document");
    break;
  case 407:
    task = getTask("bring meal");
    break;
  case 408:
    task = getTask("take meal");
    break;
  case 409:
    task = getTask("send meal");
    break;
  case 410:
    task = getTask("bring coffee");
    break;
  case 411:
    task = getTask("take coffee");
    break;
  case 412:
    task = getTask("send coffee");
    break;
  case 413:
    task = getTask("bring gallon");
    break;
  case 414:
    task = getTask("take gallon");
    break;
  case 415:
    task = getTask("send gallon");
    break;
  }
  return task;
}

/**
 * @brief DatabaseInterface::getUser
 * @param id
 * @return
 */
agents::User DatabaseInterface::getUser(int id) const
{
  agents::User user;
  switch (id)
  {
  case 101:
    user = getUser("adrianohrl");
    break;
  case 103:
    user = getUser("luis");
    break;
  case 104:
    user = getUser("heverton");
    break;
  case 105:
    user = getUser("audeliano");
    break;
  }
  return user;
}

/**
 * @brief DatabaseInterface::getComputer
 * @param hostname
 * @return
 */
agents::Computer DatabaseInterface::getComputer(std::string hostname) const
{
  // realizar consultas no DB para preencher o objeto a partir do seu hostname e
  // enviá-lo
  // Simulado, por enquanto
  int id(0);
  bool mobile(false);
  double x(0.0);
  double y(0.0);
  double theta(0.0);
  if (hostname == "adrianohrl-pc")
  {
    id = 201;
    x = -1.0;
    y = .5;
    theta = -2.5;
  }
  else if (hostname == "luis-pc")
  {
    id = 203;
    mobile = true;
    x = .6;
    y = -6.4;
    theta = -7.3;
  }
  else if (hostname == "heverton-pc")
  {
    id = 204;
    x = 1.68;
    y = 6.45;
    theta = -.3;
  }
  else if (hostname == "audeliano-pc")
  {
    id = 205;
    x = -1.68;
    y = -6.45;
    theta = -.3;
  }
  return agents::Computer(id, hostname, mobile, x, y, theta);
}

/**
 * @brief DatabaseInterface::getPerson
 * @param name
 * @return
 */
agents::Person DatabaseInterface::getPerson(std::string name) const
{
  int id(0);
  agents::HierarchyLevelEnum hierarchy_level(agents::HierarchyLevels::getDefault());
  // realizar consultas no DB para preencher o objeto a partir do seu id e
  // enviá-lo
  if (name == "Adriano Henrique Rossette Leite")
  {
    id = 101;
    hierarchy_level = agents::HierarchyLevels::toEnumerated(3);
  }
  else if (name == "Luís Victor Pessiqueli Bonin")
  {
    id = 103;
    hierarchy_level = agents::HierarchyLevels::toEnumerated(6);
  }
  else if (name == "Héverton Soares")
  {
    id = 104;
    hierarchy_level = agents::levels::SENIOR_MANAGER;
  }
  else if (name == "Audeliano Li")
  {
    id = 105;
    hierarchy_level = agents::HierarchyLevels::toEnumerated(2);
  }
  return agents::Person(id, name, hierarchy_level);
}

/**
 * @brief DatabaseInterface::getRobot
 * @param hostname
 * @return
 */
agents::Robot DatabaseInterface::getRobot(std::string hostname) const
{
  // realizar consultas no DB para preencher o objeto a partir do seu hostname e
  // enviá-lo
  // Simulado, por enquanto
  int id(0);
  bool holonomic(false);
  bool mobile(true);
  double x(0.0);
  double y(0.0);
  double theta(0.0);
  std::vector<tasks::Skill*> skills;
  if (hostname == "robotino1")
  {
    id = 301;
    holonomic = true;
    x = -1.0;
    y = .5;
    theta = -2.5;
    skills.push_back(new tasks::Skill("velocity", tasks::levels::HIGH));
    skills.push_back(new tasks::Skill("strength", tasks::levels::LOW));
    skills.push_back(new tasks::Skill("flexibility", tasks::levels::RESOURCEFUL));
    skills.push_back(new tasks::Skill("smoothness", tasks::levels::HIGH));
  }
  else if (hostname == "robotino2")
  {
    id = 302;
    holonomic = true;
    skills.push_back(new tasks::Skill("velocity", tasks::levels::LOW));
    skills.push_back(new tasks::Skill("strength", tasks::levels::LOW));
    skills.push_back(new tasks::Skill("flexibility", tasks::levels::HIGH));
    skills.push_back(new tasks::Skill("smoothness", tasks::levels::RESOURCEFUL));
  }
  else if (hostname == "robotino3")
  {
    id = 303;
    holonomic = true;
    x = .6;
    y = -6.4;
    theta = -7.3;
    skills.push_back(new tasks::Skill("velocity", tasks::levels::MODERATE));
    skills.push_back(new tasks::Skill("strength", tasks::levels::LOW));
    skills.push_back(new tasks::Skill("flexibility", tasks::levels::MODERATE));
    skills.push_back(new tasks::Skill("smoothness", tasks::levels::HIGH));
  }
  else if (hostname == "p3_dx")
  {
    id = 304;
    x = 1.68;
    y = 6.45;
    theta = -.3;
    skills.push_back(new tasks::Skill("velocity", tasks::levels::HIGH));
    skills.push_back(new tasks::Skill("strength", tasks::levels::RESOURCEFUL));
    skills.push_back(new tasks::Skill("flexibility", tasks::levels::LOW));
    skills.push_back(new tasks::Skill("smoothness", tasks::levels::MODERATE));
  }
  agents::Robot robot(id, hostname, holonomic, mobile, x, y, theta);
  robot.setSkills(skills);
  return robot;
}

/**
 * IMPLEMENTAR
 */
tasks::Task DatabaseInterface::getTask(std::string name) const
{
  tasks::Task task;
  if (name == "bring water")
  {
    task.setId(401);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::HIGH));
    task.addSkill(new tasks::Skill("smoothness", tasks::levels::RESOURCEFUL));
  }
  else if (name == "take water")
  {
    task.setId(402);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::HIGH));
    task.addSkill(new tasks::Skill("smoothness", tasks::levels::RESOURCEFUL));
  }
  else if (name == "send water")
  {
    task.setId(403);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::HIGH));
    task.addSkill(new tasks::Skill("smoothness", tasks::levels::RESOURCEFUL));
  }
  else if (name == "bring document")
  {
    task.setId(404);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::HIGH));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::MODERATE));
  }
  else if (name == "take document")
  {
    task.setId(405);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::HIGH));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::MODERATE));
  }
  else if (name == "send document")
  {
    task.setId(406);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::HIGH));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::MODERATE));
  }
  else if (name == "bring meal")
  {
    task.setId(407);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::HIGH));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::LOW));
  }
  else if (name == "take meal")
  {
    task.setId(408);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::HIGH));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::LOW));
  }
  else if (name == "send meal")
  {
    task.setId(409);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::HIGH));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::LOW));
  }
  else if (name == "bring coffee")
  {
    task.setId(410);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::MODERATE));
    task.addSkill(new tasks::Skill("smoothness", tasks::levels::HIGH));
  }
  else if (name == "take coffee")
  {
    task.setId(411);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::MODERATE));
    task.addSkill(new tasks::Skill("smoothness", tasks::levels::HIGH));
  }
  else if (name == "send coffee")
  {
    task.setId(412);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("strength", tasks::levels::LOW));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::MODERATE));
    task.addSkill(new tasks::Skill("smoothness", tasks::levels::HIGH));
  }
  else if (name == "bring gallon")
  {
    task.setId(413);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::MODERATE));
    task.addSkill(new tasks::Skill("strength", tasks::levels::RESOURCEFUL));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::LOW));
  }
  else if (name == "take gallon")
  {
    task.setId(414);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::MODERATE));
    task.addSkill(new tasks::Skill("strength", tasks::levels::RESOURCEFUL));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::LOW));
  }
  else if (name == "send gallon")
  {
    task.setId(415);
    task.setName(name);
    task.addSkill(new tasks::Skill("velocity", tasks::levels::MODERATE));
    task.addSkill(new tasks::Skill("strength", tasks::levels::RESOURCEFUL));
    task.addSkill(new tasks::Skill("flexibility", tasks::levels::LOW));
  }
  return task;
}

/**
 * @brief DatabaseInterface::getUser
 * @param login_name
 * @return
 */
agents::User DatabaseInterface::getUser(std::string login_name) const
{
  int id(0);
  std::string name("");
  agents::HierarchyLevelEnum hierarchy_level(agents::HierarchyLevels::getDefault());
  agents::Computer* computer = NULL;
  // realizar consultas no DB para preencher o objeto a partir do seu login_name
  // e enviá-lo
  // Simulado, por enquanto
  if (login_name == "adrianohrl")
  {
    id = 101;
    name = "Adriano Henrique Rossette Leite";
    hierarchy_level = agents::levels::EMPLOYEE;
    computer = new agents::Computer(getComputer("adrianohrl-pc"));
  }
  else if (login_name == "luis")
  {
    id = 103;
    name = "Luís Victor Pessiqueli Bonin";
    hierarchy_level = agents::levels::SUPERVISOR;
    computer = new agents::Computer(getComputer("luis-pc"));
  }
  else if (login_name == "heverton")
  {
    id = 104;
    name = "Héverton Soares";
    hierarchy_level = agents::levels::SENIOR_MANAGER;
    computer = new agents::Computer(getComputer("heverton-pc"));
  }
  else if (login_name == "audeliano")
  {
    id = 105;
    name = "Audeliano Li";
    hierarchy_level = agents::levels::CEO;
    computer = new agents::Computer(getComputer("audeliano-pc"));
  }
  return agents::User(id, name, hierarchy_level, login_name, computer);
}

/**
 * @brief DatabaseInterface::generateNewAgentId
 * @return
 */
int DatabaseInterface::generateNewAgentId()
{
  agents_counter_++;
  return agents_counter_;
}

/**
 * @brief DatabaseInterface::generateNewAllocationId
 * @return
 */
int DatabaseInterface::generateNewAllocationId()
{
  allocations_counter_++;
  return allocations_counter_;
}

/**
 * @brief DatabaseInterface::generateNewPlaceId
 * @return
 */
int DatabaseInterface::generateNewPlaceId()
{
  places_counter_++;
  return places_counter_;
}

/**
 * @brief DatabaseInterface::generateNewResourceId
 * @return
 */
int DatabaseInterface::generateNewResourceId()
{
  resources_counter_++;
  return resources_counter_;
}

/**
 * @brief DatabaseInterface::generateNewSkillId
 * @return
 */
int DatabaseInterface::generateNewSkillId()
{
  skills_counter_++;
  return skills_counter_;
}

/**
 * @brief DatabaseInterface::generateNewTaskId
 * @return
 */
int DatabaseInterface::generateNewTaskId()
{
  tasks_counter_++;
  return tasks_counter_;
}
}
}
