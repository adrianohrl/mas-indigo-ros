/**
 *  This source file implements the SystemDatabaseInterfaceNode class, which is
 *based on the ROSNode helper class. It controls the
 *system_database_interface_node.
 *
 *  Version: 1.4.0
 *  Created on: 01/04/2016
 *  Modified on: 20/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Christiano Henrique Rezende (c.h.rezende@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas_database/system_database_interface_node.h"

// using typename mas::database::DatabaseInterface;
// using typename mas::database::EntityTypes;
using namespace mas::database; // because of entity type enums.

namespace mas_database
{

/**
 * @brief SystemDatabaseInterfaceNode::SystemDatabaseInterfaceNode
 * @param nh
 */
SystemDatabaseInterfaceNode::SystemDatabaseInterfaceNode(ros::NodeHandle* nh)
    : ROSNode(nh, 10)
{
  generate_new_id_srv_ = nh->advertiseService(
      "/generate_new_id", &SystemDatabaseInterfaceNode::generateNewId, this);
  get_computer_srv_ = nh->advertiseService(
      "/get_computer", &SystemDatabaseInterfaceNode::getComputer, this);
  get_person_srv_ = nh->advertiseService(
      "/get_person", &SystemDatabaseInterfaceNode::getPerson, this);
  get_robot_srv_ = nh->advertiseService(
      "/get_robot", &SystemDatabaseInterfaceNode::getRobot, this);
  get_task_srv_ = nh->advertiseService(
      "/get_task", &SystemDatabaseInterfaceNode::getTask, this);
  get_user_srv_ = nh->advertiseService(
      "/get_user", &SystemDatabaseInterfaceNode::getUser, this);
  validate_srv_ = nh->advertiseService(
      "/validate_password",
      &SystemDatabaseInterfaceNode::validatePasswordCallback, this);
}

/**
 * @brief SystemDatabaseInterfaceNode::~SystemDatabaseInterfaceNode
 */
SystemDatabaseInterfaceNode::~SystemDatabaseInterfaceNode()
{
  generate_new_id_srv_.shutdown();
  get_computer_srv_.shutdown();
  get_person_srv_.shutdown();
  get_robot_srv_.shutdown();
  get_task_srv_.shutdown();
  get_user_srv_.shutdown();
  validate_srv_.shutdown();
}

/**
 * @brief SystemDatabaseInterfaceNode::controlLoop
 */
void SystemDatabaseInterfaceNode::controlLoop() {}

/**
 * @brief SystemDatabaseInterfaceNode::generateNewId
 * @param request
 * @param response
 * @return
 */
bool SystemDatabaseInterfaceNode::generateNewId(
    mas_srvs::GenerateNewId::Request& request,
    mas_srvs::GenerateNewId::Response& response)
{
  switch (EntityTypes::toEnumerated(request.type))
  {
  case types::AGENT:
    response.id = db_.generateNewAgentId();
    break;
  case types::ALLOCATION:
    response.id = db_.generateNewAllocationId();
    break;
  case types::PLACE:
    response.id = db_.generateNewPlaceId();
    break;
  case types::RESOURCE:
    response.id = db_.generateNewResourceId();
    break;
  case types::SKILL:
    response.id = db_.generateNewSkillId();
    break;
  case types::TASK:
    response.id = db_.generateNewTaskId();
    break;
  default:
    return false;
  }
  return true;
}

/**
 * @brief SystemDatabaseInterfaceNode::getComputer
 * @param request
 * @param response
 * @return
 */
bool SystemDatabaseInterfaceNode::getComputer(
    mas_srvs::GetComputer::Request& request,
    mas_srvs::GetComputer::Response& response)
{
  if (!db_.isComputerRegistered(request.hostname))
  {
    response.message = "There is no computer hostnamed as " + request.hostname +
                       " registered in database!!!";
    response.valid = false;
    return response.valid;
  }
  response.computer = db_.getComputer(request.hostname).to_msg();
  response.message = request.hostname + " is registered!!!";
  response.valid = true;
  return response.valid;
}

/**
 * @brief SystemDatabaseInterfaceNode::getPerson
 * @param request
 * @param response
 * @return
 */
bool SystemDatabaseInterfaceNode::getPerson(
    mas_srvs::GetPerson::Request& request,
    mas_srvs::GetPerson::Response& response)
{
  if (!db_.isPersonRegistered(request.name))
  {
    response.message = "There is no person named as " + request.name +
                       " registered in database!!!";
    response.valid = false;
    return response.valid;
  }
  std::string user_login_name(db_.getUserLoginName(request.name));
  if (!user_login_name.empty())
  {
    response.person = db_.getUser(user_login_name).to_msg();
  }
  else
  {
    response.person = db_.getPerson(request.name).to_msg();
  }
  response.message = request.name + " is registered!!!";
  response.valid = true;
  return response.valid;
}

/**
 * @brief SystemDatabaseInterfaceNode::getRobot
 * @param request
 * @param response
 * @return
 */
bool SystemDatabaseInterfaceNode::getRobot(
    mas_srvs::GetRobot::Request& request,
    mas_srvs::GetRobot::Response& response)
{
  if (!db_.isRobotRegistered(request.hostname))
  {
    response.message = "There is no robot hostnamed as " + request.hostname +
                       " registered in database!!!";
    response.valid = false;
    return response.valid;
  }
  response.robot = db_.getRobot(request.hostname).to_msg();
  response.message = request.hostname + " is registered!!!";
  response.valid = true;
  return response.valid;
}

/**
 * @brief SystemDatabaseInterfaceNode::getTask
 * @param request
 * @param response
 * @return
 */
bool SystemDatabaseInterfaceNode::getTask(mas_srvs::GetTask::Request& request,
                                          mas_srvs::GetTask::Response& response)
{
  if (!db_.isTaskRegistered(request.name))
  {
    response.message = "There is no task named as " + request.name +
                       " registered in database!!!";
    response.valid = false;
    return response.valid;
  }
  response.task = db_.getTask(request.name).to_msg();
  response.message = request.name + " is registered!!!";
  response.valid = true;
  return response.valid;
}

/**
 * @brief SystemDatabaseInterfaceNode::getUser
 * @param request
 * @param response
 * @return
 */
bool SystemDatabaseInterfaceNode::getUser(mas_srvs::GetUser::Request& request,
                                          mas_srvs::GetUser::Response& response)
{
  if (!db_.isUserRegistered(request.name))
  {
    response.message = "There is no user named as " + request.name +
                       " registered in database!!!";
    response.valid = false;
    return response.valid;
  }
  std::string user_login_name(db_.getUserLoginName(request.name));
  response.user = db_.getUser(user_login_name).to_msg();
  response.message = request.name + " is registered!!!";
  response.valid = true;
  return response.valid;
}

/**
 * @brief SystemDatabaseInterfaceNode::validatePasswordCallback
 * @param request
 * @param response
 * @return
 */
bool SystemDatabaseInterfaceNode::validatePasswordCallback(
    mas_srvs::ValidatePassword::Request& request,
    mas_srvs::ValidatePassword::Response& response)
{
  std::string password("");
  if (request.login_name == "adrianohrl")
  {
    password = "teste123";
  }
  else if (request.login_name == "christiano")
  {
    password = "teste1234";
  }
  else if (request.login_name == "luis")
  {
    password = "teste12345";
  }
  else if (request.login_name == "heverton")
  {
    password = "teste123456";
  }
  else if (request.login_name == "audeliano")
  {
    password = "teste1234567";
  }
  // nos passos acima, será utilizado uma consulta no DB para pegar a senha
  // encriptografada cujo login_name é desejado
  response.valid = false;
  if (password.empty())
  {
    response.message =
        "There is no voice commander registered with this login name!!!";
  }
  else if (password == request.password)
  {
    response.user = db_.getUser(request.login_name).to_msg();
    response.message = "Valid password!!!";
    response.valid = true;
  }
  else
  {
    response.message = "Invalid password!!!";
  }
  return response.valid;
}
}
