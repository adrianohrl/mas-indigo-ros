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
mrta_vc::SystemDatabaseInterfaceNode::SystemDatabaseInterfaceNode(ros::NodeHandle nh) : unifei::expertinos::mrta_vc::system::DatabaseInterface(), nh_(nh)
{
	validate_srv_ = nh_.advertiseService("validate_password", &mrta_vc::SystemDatabaseInterfaceNode::validatePasswordCallback, this);
}

/**
 * Destructor
 */
mrta_vc::SystemDatabaseInterfaceNode::~SystemDatabaseInterfaceNode()
{
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
	if (password != request.password)
	{
		response.message = "Invalid password!!!";
	}
	else 
	{
		response.voice_commander = unifei::expertinos::mrta_vc::system::DatabaseInterface::getVoiceCommander(request.login_name).toMsg();
		response.message = "Valid password!!!";
		response.valid = true;
	}
	return response.valid;
}
