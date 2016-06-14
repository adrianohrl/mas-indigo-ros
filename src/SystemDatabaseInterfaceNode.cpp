/**
 *  SystemDatabaseInterfaceNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Christiano Henrique Rezende (c.h.rezende@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemDatabaseInterfaceNode.h"

/**
 * Constructor
 */
mrta_vc::SystemDatabaseInterfaceNode::SystemDatabaseInterfaceNode(ros::NodeHandle nh) : nh_(nh)
{
	generate_new_id_srv_ = nh_.advertiseService("/generate_new_id", &mrta_vc::SystemDatabaseInterfaceNode::generateNewId, this);
  get_computer_srv_ = nh_.advertiseService("/get_computer", &mrta_vc::SystemDatabaseInterfaceNode::getComputer, this);
  get_person_srv_ = nh_.advertiseService("/get_person", &mrta_vc::SystemDatabaseInterfaceNode::getPerson, this);
  get_robot_srv_ = nh_.advertiseService("/get_robot", &mrta_vc::SystemDatabaseInterfaceNode::getRobot, this);
  get_task_srv_ = nh_.advertiseService("/get_task", &mrta_vc::SystemDatabaseInterfaceNode::getTask, this);
  get_user_srv_ = nh_.advertiseService("/get_user", &mrta_vc::SystemDatabaseInterfaceNode::getUser, this);
	validate_srv_ = nh_.advertiseService("/validate_password", &mrta_vc::SystemDatabaseInterfaceNode::validatePasswordCallback, this);
}

/**
 * Destructor
 */
mrta_vc::SystemDatabaseInterfaceNode::~SystemDatabaseInterfaceNode()
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
 * 
 */
void mrta_vc::SystemDatabaseInterfaceNode::spin() 
{
	ROS_INFO("System Database Interface Node is up and running!!!");
	ros::Rate loop_rate(10.0);
	while (nh_.ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/**
 *
 */
bool mrta_vc::SystemDatabaseInterfaceNode::generateNewId(mrta_vc::GenerateNewId::Request &request, mrta_vc::GenerateNewId::Response &response)
{
	switch (unifei::expertinos::mrta_vc::system::EntityTypes::toEnumerated(request.type))
	{
		case unifei::expertinos::mrta_vc::system::types::AGENT:
			response.id = unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewAgentId();
			break;
		case unifei::expertinos::mrta_vc::system::types::ALLOCATION:
			response.id = unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewAllocationId();
			break;
		case unifei::expertinos::mrta_vc::system::types::PLACE:
			response.id = unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewPlaceId();
			break;
		case unifei::expertinos::mrta_vc::system::types::RESOURCE:
			response.id = unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewResourceId();
			break;
		case unifei::expertinos::mrta_vc::system::types::SKILL:
			response.id = unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewSkillId();
			break;
		case unifei::expertinos::mrta_vc::system::types::TASK:
			response.id = unifei::expertinos::mrta_vc::system::DatabaseInterface::generateNewTaskId();
			break;
		default:
			return false;
	}
	return true;
}

/**
 *
 */
bool mrta_vc::SystemDatabaseInterfaceNode::getComputer(mrta_vc::GetComputer::Request& request, mrta_vc::GetComputer::Response& response)
{
  if (!unifei::expertinos::mrta_vc::system::DatabaseInterface::isComputerRegistered(request.hostname))
  {
    response.message = "There is no computer hostnamed as " + request.hostname + " registered in database!!!";
    response.valid = false;
    return response.valid;
  }
  response.computer = unifei::expertinos::mrta_vc::system::DatabaseInterface::getComputer(request.hostname).toMsg();
  response.message = request.hostname + " is registered!!!";
  response.valid = true;
  return response.valid;
}

/**
 *
 */
bool mrta_vc::SystemDatabaseInterfaceNode::getPerson(mrta_vc::GetPerson::Request& request, mrta_vc::GetPerson::Response& response)
{
  if (!unifei::expertinos::mrta_vc::system::DatabaseInterface::isPersonRegistered(request.name))
  {
    response.message = "There is no person named as " + request.name + " registered in database!!!";
    response.valid = false;
    return response.valid;
  }
  std::string user_login_name = unifei::expertinos::mrta_vc::system::DatabaseInterface::getUserLoginName(request.name);
  if (user_login_name != "")
  {
    response.person = unifei::expertinos::mrta_vc::system::DatabaseInterface::getUser(user_login_name).toMsg();
  }
  else
  {
    response.person = unifei::expertinos::mrta_vc::system::DatabaseInterface::getPerson(request.name).toMsg();
  }
  response.message = request.name + " is registered!!!";
  response.valid = true;
  return response.valid;
}

/**
 *
 */
bool mrta_vc::SystemDatabaseInterfaceNode::getRobot(mrta_vc::GetRobot::Request& request, mrta_vc::GetRobot::Response& response)
{
  if (!unifei::expertinos::mrta_vc::system::DatabaseInterface::isRobotRegistered(request.hostname))
  {
    response.message = "There is no robot hostnamed as " + request.hostname + " registered in database!!!";
    response.valid = false;
    return response.valid;
  }
  response.robot = unifei::expertinos::mrta_vc::system::DatabaseInterface::getRobot(request.hostname).toMsg();
  response.message = request.hostname + " is registered!!!";
  response.valid = true;
  return response.valid;
}

/**
 *
 */
bool mrta_vc::SystemDatabaseInterfaceNode::getTask(mrta_vc::GetTask::Request& request, mrta_vc::GetTask::Response& response)
{
  if (!unifei::expertinos::mrta_vc::system::DatabaseInterface::isTaskRegistered(request.name))
  {
    response.message = "There is no task named as " + request.name + " registered in database!!!";
    response.valid = false;
    return response.valid;
  }
  response.task = unifei::expertinos::mrta_vc::system::DatabaseInterface::getTask(request.name).toMsg();
  response.message = request.name + " is registered!!!";
  response.valid = true;
  return response.valid;
}

/**
 *
 */
bool mrta_vc::SystemDatabaseInterfaceNode::getUser(mrta_vc::GetUser::Request& request, mrta_vc::GetUser::Response& response)
{
  if (!unifei::expertinos::mrta_vc::system::DatabaseInterface::isUserRegistered(request.name))
  {
    response.message = "There is no user named as " + request.name + " registered in database!!!";
    response.valid = false;
    return response.valid;
  }
  std::string user_login_name = unifei::expertinos::mrta_vc::system::DatabaseInterface::getUserLoginName(request.name);
  response.user = unifei::expertinos::mrta_vc::system::DatabaseInterface::getUser(user_login_name).toMsg();
  response.message = request.name + " is registered!!!";
  response.valid = true;
  return response.valid;
}

/**
 * 
 */
bool mrta_vc::SystemDatabaseInterfaceNode::validatePasswordCallback(mrta_vc::ValidatePassword::Request& request, mrta_vc::ValidatePassword::Response& response)
{
	std::string password;
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
	else
	{
		password = "";
	}
	// nos passos acima, será utilizado uma consulta no DB para pegar a senha encriptografada cujo login_name é desejado
	response.valid = false;
	if (password == "") 
	{
		response.message = "There is no voice commander registered with this login name!!!";
	}
	else if (password == request.password)
	{
    response.user = unifei::expertinos::mrta_vc::system::DatabaseInterface::getUser(request.login_name).toMsg();
		response.message = "Valid password!!!";
		response.valid = true;
	}
	else 
	{
		response.message = "Invalid password!!!";
	}
	return response.valid;
}
