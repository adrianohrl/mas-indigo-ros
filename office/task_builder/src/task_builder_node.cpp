/**
 *  This source file implements the main function that calls the Task Builder
 *node controller.
 *
 *  Version: 1.4.0
 *  Created on: 01/04/2016
 *  Modified on: 16/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/task_builder_node.h"

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_builder_node");
  ros::NodeHandle nh;
  task_builder::TaskBuilderNode node(&nh);
  node.spin();
  return 0;
}
