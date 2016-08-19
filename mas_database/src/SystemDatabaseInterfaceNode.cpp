/**
 *  SystemDatabaseInterfaceNode.cpp
 *
 *  Version: 1.2.4
 *  Created on: 01/04/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Christiano Henrique Rezende (c.h.rezende@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas_database/SystemDatabaseInterfaceNode.h"

//using typename mas::database::DatabaseInterface;
//using typename mas::database::EntityTypes;
using namespace mas::database; // because of entity type enums.

namespace mas_database
{

	/**
	 * Constructor
	 */
	SystemDatabaseInterfaceNode::SystemDatabaseInterfaceNode(ros::NodeHandle nh) : nh_(nh)
	{
		generate_new_id_srv_ = nh_.advertiseService("/generate_new_id", &SystemDatabaseInterfaceNode::generateNewId, this);
		get_computer_srv_ = nh_.advertiseService("/get_computer", &SystemDatabaseInterfaceNode::getComputer, this);
		get_person_srv_ = nh_.advertiseService("/get_person", &SystemDatabaseInterfaceNode::getPerson, this);
		get_robot_srv_ = nh_.advertiseService("/get_robot", &SystemDatabaseInterfaceNode::getRobot, this);
		get_task_srv_ = nh_.advertiseService("/get_task", &SystemDatabaseInterfaceNode::getTask, this);
		get_user_srv_ = nh_.advertiseService("/get_user", &SystemDatabaseInterfaceNode::getUser, this);
		validate_srv_ = nh_.advertiseService("/validate_password", &SystemDatabaseInterfaceNode::validatePasswordCallback, this);
	}

	/**
	 * Destructor
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
	 * 
	 */
	void SystemDatabaseInterfaceNode::spin() 
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
	bool SystemDatabaseInterfaceNode::generateNewId(mas_srvs::GenerateNewId::Request &request, mas_srvs::GenerateNewId::Response &response)
	{
		switch (EntityTypes::toEnumerated(request.type))
		{
			case types::AGENT:
				response.id = DatabaseInterface::generateNewAgentId();
				break;
			case types::ALLOCATION:
				response.id = DatabaseInterface::generateNewAllocationId();
				break;
			case types::PLACE:
				response.id = DatabaseInterface::generateNewPlaceId();
				break;
			case types::RESOURCE:
				response.id = DatabaseInterface::generateNewResourceId();
				break;
			case types::SKILL:
				response.id = DatabaseInterface::generateNewSkillId();
				break;
			case types::TASK:
				response.id = DatabaseInterface::generateNewTaskId();
				break;
			default:
				return false;
		}
		return true;
	}

	/**
	 *
	 */
	bool SystemDatabaseInterfaceNode::getComputer(mas_srvs::GetComputer::Request& request, mas_srvs::GetComputer::Response& response)
	{
	  if (!DatabaseInterface::isComputerRegistered(request.hostname))
	  {
	    response.message = "There is no computer hostnamed as " + request.hostname + " registered in database!!!";
	    response.valid = false;
	    return response.valid;
	  }
	  response.computer = DatabaseInterface::getComputer(request.hostname).toMsg();
	  response.message = request.hostname + " is registered!!!";
	  response.valid = true;
	  return response.valid;
	}

	/**
	 *
	 */
	bool SystemDatabaseInterfaceNode::getPerson(mas_srvs::GetPerson::Request& request, mas_srvs::GetPerson::Response& response)
	{
	  if (!DatabaseInterface::isPersonRegistered(request.name))
	  {
	    response.message = "There is no person named as " + request.name + " registered in database!!!";
	    response.valid = false;
	    return response.valid;
	  }
	  std::string user_login_name = DatabaseInterface::getUserLoginName(request.name);
	  if (user_login_name != "")
	  {
	    response.person = DatabaseInterface::getUser(user_login_name).toMsg();
	  }
	  else
	  {
	    response.person = DatabaseInterface::getPerson(request.name).toMsg();
	  }
	  response.message = request.name + " is registered!!!";
	  response.valid = true;
	  return response.valid;
	}

	/**
	 *
	 */
	bool SystemDatabaseInterfaceNode::getRobot(mas_srvs::GetRobot::Request& request, mas_srvs::GetRobot::Response& response)
	{
	  if (!DatabaseInterface::isRobotRegistered(request.hostname))
	  {
	    response.message = "There is no robot hostnamed as " + request.hostname + " registered in database!!!";
	    response.valid = false;
	    return response.valid;
	  }
	  response.robot = DatabaseInterface::getRobot(request.hostname).toMsg();
	  response.message = request.hostname + " is registered!!!";
	  response.valid = true;
	  return response.valid;
	}

	/**
	 *
	 */
	bool SystemDatabaseInterfaceNode::getTask(mas_srvs::GetTask::Request& request, mas_srvs::GetTask::Response& response)
	{
	  if (!DatabaseInterface::isTaskRegistered(request.name))
	  {
	    response.message = "There is no task named as " + request.name + " registered in database!!!";
	    response.valid = false;
	    return response.valid;
	  }
	  response.task = DatabaseInterface::getTask(request.name).toMsg();
	  response.message = request.name + " is registered!!!";
	  response.valid = true;
	  return response.valid;
	}

	/**
	 *
	 */
	bool SystemDatabaseInterfaceNode::getUser(mas_srvs::GetUser::Request& request, mas_srvs::GetUser::Response& response)
	{
	  if (!DatabaseInterface::isUserRegistered(request.name))
	  {
	    response.message = "There is no user named as " + request.name + " registered in database!!!";
	    response.valid = false;
	    return response.valid;
	  }
	  std::string user_login_name = DatabaseInterface::getUserLoginName(request.name);
	  response.user = DatabaseInterface::getUser(user_login_name).toMsg();
	  response.message = request.name + " is registered!!!";
	  response.valid = true;
	  return response.valid;
	}

	/**
	 * 
	 */
	bool SystemDatabaseInterfaceNode::validatePasswordCallback(mas_srvs::ValidatePassword::Request& request, mas_srvs::ValidatePassword::Response& response)
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
	    response.user = DatabaseInterface::getUser(request.login_name).toMsg();
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
