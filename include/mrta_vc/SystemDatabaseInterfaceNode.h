/**
 *  SystemDatabaseInterfaceNode.h
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Christiano Henrique Rezende (c.h.rezende@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_DATABASE_INTERFACE_NODE_H_
#define SYSTEM_DATABASE_INTERFACE_NODE_H_

#include <ros/ros.h>
#include "mrta_vc/GenerateNewId.h"
#include "mrta_vc/GetComputer.h"
#include "mrta_vc/GetPerson.h"
#include "mrta_vc/GetRobot.h"
#include "mrta_vc/GetTask.h"
#include "mrta_vc/GetUser.h"
#include "mrta_vc/ValidatePassword.h"
#include "unifei/expertinos/mrta_vc/system/DatabaseInterface.h"
#include "unifei/expertinos/mrta_vc/system/EntityTypes.h"

namespace mrta_vc 
{

  class SystemDatabaseInterfaceNode : public unifei::expertinos::mrta_vc::system::DatabaseInterface
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

		bool generateNewId(mrta_vc::GenerateNewId::Request& request, mrta_vc::GenerateNewId::Response& response);
    bool getComputer(mrta_vc::GetComputer::Request& request, mrta_vc::GetComputer::Response& response);
    bool getPerson(mrta_vc::GetPerson::Request& request, mrta_vc::GetPerson::Response& response);
    bool getRobot(mrta_vc::GetRobot::Request& request, mrta_vc::GetRobot::Response& response);
    bool getTask(mrta_vc::GetTask::Request& request, mrta_vc::GetTask::Response& response);
    bool getUser(mrta_vc::GetUser::Request& request, mrta_vc::GetUser::Response& response);
    bool validatePasswordCallback(mrta_vc::ValidatePassword::Request& request, mrta_vc::ValidatePassword::Response& response);

	};
}

#endif /* SYSTEM_DATABASE_INTERFACE_NODE_H_ */
