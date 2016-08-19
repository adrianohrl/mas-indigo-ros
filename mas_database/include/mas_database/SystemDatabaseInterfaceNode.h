/**
 *  SystemDatabaseInterfaceNode.h
 *
 *  Version: 1.2.4
 *  Created on: 01/04/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_DATABASE_INTERFACE_NODE_H_
#define SYSTEM_DATABASE_INTERFACE_NODE_H_

#include <ros/ros.h>
#include <mas_srvs/GenerateNewId.h>
#include <mas_srvs/GetComputer.h>
#include <mas_srvs/GetPerson.h>
#include <mas_srvs/GetRobot.h>
#include <mas_srvs/GetTask.h>
#include <mas_srvs/GetUser.h>
#include <mas_srvs/ValidatePassword.h>
#include <mas/database/DatabaseInterface.h>
#include <mas/database/EntityTypes.h>

namespace mas_database 
{

	class SystemDatabaseInterfaceNode : public mas::database::DatabaseInterface
	{

	public:
		SystemDatabaseInterfaceNode(ros::NodeHandle nh);
		virtual ~SystemDatabaseInterfaceNode();

		void spin();

	private:
		ros::NodeHandle nh_;
		ros::ServiceServer generate_new_id_srv_;
		ros::ServiceServer get_computer_srv_;
		ros::ServiceServer get_person_srv_;
		ros::ServiceServer get_robot_srv_;
		ros::ServiceServer get_task_srv_;
		ros::ServiceServer get_user_srv_;
		ros::ServiceServer validate_srv_;

		bool generateNewId(mas_srvs::GenerateNewId::Request& request, mas_srvs::GenerateNewId::Response& response);
		bool getComputer(mas_srvs::GetComputer::Request& request, mas_srvs::GetComputer::Response& response);
		bool getPerson(mas_srvs::GetPerson::Request& request, mas_srvs::GetPerson::Response& response);
		bool getRobot(mas_srvs::GetRobot::Request& request, mas_srvs::GetRobot::Response& response);
		bool getTask(mas_srvs::GetTask::Request& request, mas_srvs::GetTask::Response& response);
		bool getUser(mas_srvs::GetUser::Request& request, mas_srvs::GetUser::Response& response);
		bool validatePasswordCallback(mas_srvs::ValidatePassword::Request& request, mas_srvs::ValidatePassword::Response& response);

	};
}

#endif /* SYSTEM_DATABASE_INTERFACE_NODE_H_ */
