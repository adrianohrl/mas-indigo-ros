/**
 *  This source file implements the main function that calls the System Database
 *Interface node controller.
 *
 *  Version: 1.4.0
 *  Created on: 01/04/2016
 *  Modified on: 20/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas_database/system_database_interface_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "system_database_interface_node");
  ros::NodeHandle nh;
  mas_database::SystemDatabaseInterfaceNode node(&nh);
  node.spin();
  return 0;
}
