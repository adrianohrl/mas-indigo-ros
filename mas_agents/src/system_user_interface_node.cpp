/**
 *  This source file implements the main function that calls the System User
 *Interface node controller.
 *
 *  Version: 1.4.0
 *  Created on: 26/03/2016
 *  Modified on: 16/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas_agents/system_user_interface_node.h"

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "system_user_interface_node");
  ros::NodeHandle nh;
  mas_agents::SystemUserInterfaceNode node(&nh);
  node.spin();
  return 0;
}
