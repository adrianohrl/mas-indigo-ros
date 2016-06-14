/**
 *  DatabaseInterface.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 21/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Christiano Henrique Rezende (c.h.rezende@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/system/DatabaseInterface.h"

/**
 *
 */
unifei::expertinos::mrta_vc::system::DatabaseInterface::DatabaseInterface() 
{	
	agents_counter_ = 0;
	allocations_counter_ = 0;
	places_counter_ = 0;
	resources_counter_ = 0;
	skills_counter_ = 0;
	tasks_counter_ = 0;
}

/**
 *
 */
unifei::expertinos::mrta_vc::system::DatabaseInterface::~DatabaseInterface() 
{
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::system::DatabaseInterface::isComputerRegistered(std::string hostname)
{
  return hostname == "adrianohrl-pc" || hostname == "christiano-pc" || hostname == "luis-pc" || hostname == "heverton-pc" || hostname == "audeliano-pc";
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::system::DatabaseInterface::isPersonRegistered(std::string name)
{
  return name == "Adriano Henrique Rossette Leite" || name == "Christiano Henrique Rezende" || name == "Luís Victor Pessiqueli Bonin" || name == "Héverton Soares" || name == "Audeliano Li";
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::system::DatabaseInterface::isRobotRegistered(std::string hostname)
{
  return hostname == "robotino1" || hostname == "robotino2" || hostname == "robotino3" || hostname == "p3_dx";
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::system::DatabaseInterface::isTaskRegistered(std::string name)
{
	return name == "bring water" || name == "take water" || name == "send water" ||
      name == "bring document" || name == "take document" || name == "send document" ||
      name == "bring meal" || name == "take meal" || name == "send meal" ||
      name == "bring coffee" || name == "take coffee" || name == "send coffee" ||
      name == "bring gallon" || name == "take gallon" || name == "send gallon";
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::system::DatabaseInterface::isUserRegistered(std::string name)
{
  return name == "Adriano Henrique Rossette Leite" || name == "Christiano Henrique Rezende" || name == "Luís Victor Pessiqueli Bonin" || name == "Héverton Soares" || name == "Audeliano Li";
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::system::DatabaseInterface::getUserLoginName(std::string name)
{
  std::string login_name = "";
  if (name == "Adriano Henrique Rossette Leite")
  {
    login_name = "adrianohrl";
  }
  else if (name == "Christiano Henrique Rezende")
  {
    login_name = "christiano";
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
 *
 */
unifei::expertinos::mrta_vc::agents::Computer unifei::expertinos::mrta_vc::system::DatabaseInterface::getComputer(int id)
{
  /*std::string hostname = "";
  bool mobile = false;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  // realizar consultas no DB para preencher o objeto a partir do seu id e enviá-lo
  return unifei::expertinos::mrta_vc::agents::Computer(id, hostname, mobile, x, y, theta);*/
  unifei::expertinos::mrta_vc::agents::Computer computer;
  switch (id)
  {
    case 201:
      computer = getComputer("adrianohrl-pc");
      break;
    case 202:
      computer = getComputer("christiano-pc");
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
 *
 */
unifei::expertinos::mrta_vc::agents::Person unifei::expertinos::mrta_vc::system::DatabaseInterface::getPerson(int id)
{
  /*std::string name = "";
  unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level = unifei::expertinos::mrta_vc::agents::levels::VISITOR;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  // realizar consultas no DB para preencher o objeto a partir do seu id e enviá-lo
  return unifei::expertinos::mrta_vc::agents::Person(id, name, hierarchy_level, x, y, theta);*/
  unifei::expertinos::mrta_vc::agents::Person person;
  switch (id)
  {
    case 101:
      person = getPerson("Adriano Henrique Rossette Leite");
      break;
    case 102:
      person = getPerson("Christiano Henrique Rezende");
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
 *
 */
unifei::expertinos::mrta_vc::agents::Robot unifei::expertinos::mrta_vc::system::DatabaseInterface::getRobot(int id)
{
  /*std::string hostname = "";
  bool holonomic = false;
  bool mobile = false;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  // realizar consultas no DB para preencher o objeto a partir do seu id e enviá-lo
  return unifei::expertinos::mrta_vc::agents::Robot(id, hostname, holonomic, mobile, x, y, theta);*/
  unifei::expertinos::mrta_vc::agents::Robot robot;
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
 * IMPLEMENTAR
 */
unifei::expertinos::mrta_vc::tasks::Task unifei::expertinos::mrta_vc::system::DatabaseInterface::getTask(int id)
{

  unifei::expertinos::mrta_vc::tasks::Task task;
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
 *
 */
unifei::expertinos::mrta_vc::agents::User unifei::expertinos::mrta_vc::system::DatabaseInterface::getUser(int id)
{
  /*std::string name = "";
	std::string login_name = "";
	int hierarchy_level = 0;
	unifei::expertinos::mrta_vc::agents::Computer computer(0, "");
	// realizar consultas no DB para preencher o objeto a partir do seu id e enviá-lo
  if (id == 100)
	{
		login_name = "adrianohrl";
		name = "Adriano Henrique Rossette Leite";
		hierarchy_level = 3;
    computer = getComputer("adrianohrl-pc");
	} 
  else if (id == 101)
	{
		login_name = "christiano";
		name = "Christiano Henrique Rezende";
		hierarchy_level = 6;
    computer = getComputer("christiano-pc");
	}
  else if (id == 102)
	{
		login_name = "luis";
		name = "Luís Victor Pessiqueli Bonin";
		hierarchy_level = 6;
    computer = getComputer("luis-pc");
	}
  else if (id == 103)
	{
		login_name = "heverton";
		name = "Héverton Soares";
		hierarchy_level = 5;
    computer = getComputer("heverton-pc");
	}
  else if (id == 104)
	{
		login_name = "audeliano";
    name = "Audeliano Li";
		hierarchy_level = 2;
    computer = getComputer("audeliano-pc");
	}
  return unifei::expertinos::mrta_vc::agents::User(id, name, unifei::expertinos::mrta_vc::agents::HierarchyLevels::toEnumerated(hierarchy_level), login_name, computer);*/
  unifei::expertinos::mrta_vc::agents::User user;
  switch (id)
  {
    case 101:
      user = getUser("adrianohrl");
      break;
    case 102:
      user = getUser("christiano");
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
 *
 */
unifei::expertinos::mrta_vc::agents::Computer unifei::expertinos::mrta_vc::system::DatabaseInterface::getComputer(std::string hostname)
{
  // realizar consultas no DB para preencher o objeto a partir do seu hostname e enviá-lo
  // Simulado, por enquanto
  unifei::expertinos::mrta_vc::agents::Computer computer;
  int id = 0;
  bool mobile = false;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  if (hostname == "adrianohrl-pc")
  {
    id = 201;
    x = -1.0;
    y = .5;
    theta = -2.5;
  }
  else if (hostname == "christiano-pc")
  {
    id = 202;
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
  return unifei::expertinos::mrta_vc::agents::Computer(id, hostname, mobile, x, y, theta);
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Person unifei::expertinos::mrta_vc::system::DatabaseInterface::getPerson(std::string name)
{
  int id = 0;
  unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level = unifei::expertinos::mrta_vc::agents::HierarchyLevels::getDefault();
  // realizar consultas no DB para preencher o objeto a partir do seu id e enviá-lo
  if (name == "Adriano Henrique Rossette Leite")
  {
    id = 101;
    hierarchy_level = unifei::expertinos::mrta_vc::agents::HierarchyLevels::toEnumerated(3);
  }
  else if (name == "Christiano Henrique Rezende")
  {
    id = 102;
    hierarchy_level = unifei::expertinos::mrta_vc::agents::levels::MIDDLE_MANAGER;
  }
  else if (name == "Luís Victor Pessiqueli Bonin")
  {
    id = 103;
    hierarchy_level = unifei::expertinos::mrta_vc::agents::HierarchyLevels::toEnumerated(6);
  }
  else if (name == "Héverton Soares")
  {
    id = 104;
    hierarchy_level = unifei::expertinos::mrta_vc::agents::levels::SENIOR_MANAGER;
  }
  else if (name == "Audeliano Li")
  {
    id = 105;
    hierarchy_level = unifei::expertinos::mrta_vc::agents::HierarchyLevels::toEnumerated(2);
  }
  return unifei::expertinos::mrta_vc::agents::Person(id, name, hierarchy_level);
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::Robot unifei::expertinos::mrta_vc::system::DatabaseInterface::getRobot(std::string hostname)
{
  // realizar consultas no DB para preencher o objeto a partir do seu hostname e enviá-lo
  // Simulado, por enquanto
  int id = 0;
  bool holonomic = false;
  bool mobile = true;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  std::vector<unifei::expertinos::mrta_vc::tasks::Skill> skills;
  if (hostname == "robotino1")
  {
    id = 301;
    holonomic = true;
    x = -1.0;
    y = .5;
    theta = -2.5;
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("smoothness", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
  }
  else if (hostname == "robotino2")
  {
    id = 302;
    holonomic = true;
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::LOW));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("smoothness", unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL));
  }
  else if (hostname == "robotino3")
  {
    id = 303;
    holonomic = true;
    x = .6;
    y = -6.4;
    theta = -7.3;
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("smoothness", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
  }
  else if (hostname == "p3_dx")
  {
    id = 304;
    x = 1.68;
    y = 6.45;
    theta = -.3;
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::LOW));
    skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("smoothness", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
  }
  unifei::expertinos::mrta_vc::agents::Robot robot(id, hostname, holonomic, mobile, x, y, theta);
  robot.setSkills(skills);
  return robot;
}

/**
 * IMPLEMENTAR
 */
unifei::expertinos::mrta_vc::tasks::Task unifei::expertinos::mrta_vc::system::DatabaseInterface::getTask(std::string name)
{
  unifei::expertinos::mrta_vc::tasks::Task task;
  if (name == "bring water")
  {
    task.setId(401);
    task.setName(name);
    task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::LOW));
    task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
    task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
    task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("smoothness", unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL));

  }
  else if (name == "take water")
  {
      task.setId(402);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("smoothness", unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL));
  }
  else if (name == "send water")
  {
      task.setId(403);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("smoothness", unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL));
  }
  else if (name == "bring document")
  {
      task.setId(404);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
  }
  else if (name == "take document")
  {
      task.setId(405);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
  }
  else if (name == "send document")
  {
      task.setId(406);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
  }
  else if (name == "bring meal")
  {
      task.setId(407);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::LOW));
  }
  else if (name == "take meal")
  {
      task.setId(408);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::LOW));
  }
  else if (name == "send meal")
  {
      task.setId(409);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::LOW));
  }
  else if (name == "bring coffee")
  {
      task.setId(410);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("smoothness", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
  }
  else if (name == "take coffee")
  {
      task.setId(411);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("smoothness", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
  }
  else if (name == "send coffee")
  {
      task.setId(412);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::LOW));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("smoothness", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
  }
  else if (name == "bring gallon")
  {
      task.setId(413);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::LOW));
  }
  else if (name == "take gallon")
  {
      task.setId(414);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::LOW));
  }
  else if (name == "send gallon")
  {
      task.setId(415);
      task.setName(name);
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::MODERATE));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL));
      task.addSkill(unifei::expertinos::mrta_vc::tasks::Skill("flexibility", unifei::expertinos::mrta_vc::tasks::levels::LOW));
  }
  return task;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::User unifei::expertinos::mrta_vc::system::DatabaseInterface::getUser(std::string login_name)
{
	int id = 0;
	std::string name = "";
  unifei::expertinos::mrta_vc::agents::HierarchyLevelEnum hierarchy_level = unifei::expertinos::mrta_vc::agents::HierarchyLevels::getDefault();
	unifei::expertinos::mrta_vc::agents::Computer computer;
	// realizar consultas no DB para preencher o objeto a partir do seu login_name e enviá-lo
	// Simulado, por enquanto
	if (login_name == "adrianohrl") 
	{
    id = 101;
		name = "Adriano Henrique Rossette Leite";
    hierarchy_level = unifei::expertinos::mrta_vc::agents::levels::EMPLOYEE;
    computer = getComputer("adrianohrl-pc");
	} 
	else if (login_name == "christiano") 
	{
    id = 102;
		name = "Christiano Henrique Rezende";
    hierarchy_level = unifei::expertinos::mrta_vc::agents::levels::MIDDLE_MANAGER;
    computer = getComputer("christiano-pc");
	}
	else if (login_name == "luis") 
	{
    id = 103;
		name = "Luís Victor Pessiqueli Bonin";
    hierarchy_level = unifei::expertinos::mrta_vc::agents::levels::SUPERVISOR;
    computer = getComputer("luis-pc");
	}
	else if (login_name == "heverton") 
	{
    id = 104;
		name = "Héverton Soares";
    hierarchy_level = unifei::expertinos::mrta_vc::agents::levels::SENIOR_MANAGER;
    computer = getComputer("heverton-pc");
	}
	else if (login_name == "audeliano") 
	{
    id = 105;
		name = "Audeliano Li";
    hierarchy_level = unifei::expertinos::mrta_vc::agents::levels::CEO;
    computer = getComputer("audeliano-pc");
	}
  return unifei::expertinos::mrta_vc::agents::User(id, name, hierarchy_level, login_name, computer);
}

/**
 *
 */
int unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewAgentId()
{
	agents_counter_++;
	return agents_counter_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewAllocationId()
{
	allocations_counter_++;
	return allocations_counter_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewPlaceId()
{
	places_counter_++;
	return places_counter_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewResourceId()
{
	resources_counter_++;
	return resources_counter_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewSkillId()
{
	skills_counter_++;
	return skills_counter_;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewTaskId()
{
	tasks_counter_++;
	return tasks_counter_;
}
