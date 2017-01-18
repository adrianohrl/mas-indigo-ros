/**
 *  This header file defines the SystemDatabaseInterfaceNode class, which is
 *based on the ROSNode helper class. It controls the
 *system_database_interface_node.
 *
 *  Version: 1.4.0
 *  Created on: 01/04/2016
 *  Modified on: 20/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _SYSTEM_DATABASE_INTERFACE_NODE_H_
#define _SYSTEM_DATABASE_INTERFACE_NODE_H_

#include <ros/ros.h>
#include <mas_srvs/GenerateNewId.h>
#include <mas_srvs/GetComputer.h>
#include <mas_srvs/GetPerson.h>
#include <mas_srvs/GetRobot.h>
#include <mas_srvs/GetTask.h>
#include <mas_srvs/GetUser.h>
#include <mas_srvs/ValidatePassword.h>
#include <mas/database/database_interface.h>
#include <mas/database/entity_types.h>
#include <utilities/ros_node.h>

namespace mas_database
{

class SystemDatabaseInterfaceNode : public utilities::ROSNode
{

public:
  SystemDatabaseInterfaceNode(ros::NodeHandle* nh);
  virtual ~SystemDatabaseInterfaceNode();

private:
  mas::database::DatabaseInterface db_;
  ros::ServiceServer generate_new_id_srv_;
  ros::ServiceServer get_computer_srv_;
  ros::ServiceServer get_person_srv_;
  ros::ServiceServer get_robot_srv_;
  ros::ServiceServer get_task_srv_;
  ros::ServiceServer get_user_srv_;
  ros::ServiceServer validate_srv_;
  virtual void controlLoop();
  bool generateNewId(mas_srvs::GenerateNewId::Request& request,
                     mas_srvs::GenerateNewId::Response& response);
  bool getComputer(mas_srvs::GetComputer::Request& request,
                   mas_srvs::GetComputer::Response& response);
  bool getPerson(mas_srvs::GetPerson::Request& request,
                 mas_srvs::GetPerson::Response& response);
  bool getRobot(mas_srvs::GetRobot::Request& request,
                mas_srvs::GetRobot::Response& response);
  bool getTask(mas_srvs::GetTask::Request& request,
               mas_srvs::GetTask::Response& response);
  bool getUser(mas_srvs::GetUser::Request& request,
               mas_srvs::GetUser::Response& response);
  bool validatePasswordCallback(mas_srvs::ValidatePassword::Request& request,
                                mas_srvs::ValidatePassword::Response& response);
};
}

#endif /* _SYSTEM_DATABASE_INTERFACE_NODE_H_ */
