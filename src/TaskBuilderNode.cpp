/**
 *  TaskBuilderNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 01/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/TaskBuilderNode.h"

/**
 * Constructor
 */
mrta_vc::TaskBuilderNode::TaskBuilderNode(ros::NodeHandle nh) : nh_(nh)
{
  task_pub_ = nh_.advertise<mrta_vc::Task>("/tasks", 2);
  abort_srv_ = nh_.advertiseService("abort", &mrta_vc::TaskBuilderNode::abort, this);
  get_person_cli_ = nh_.serviceClient<mrta_vc::GetPerson>("/get_person");
  get_user_cli_ = nh_.serviceClient<mrta_vc::GetUser>("/get_user");
}

/**
 * Destructor
 */
mrta_vc::TaskBuilderNode::~TaskBuilderNode()
{
  task_pub_.shutdown();
  abort_srv_.shutdown();
  get_person_cli_.shutdown();
  get_user_cli_.shutdown();
}

/**
 * 
 */
void mrta_vc::TaskBuilderNode::spin() 
{
	ROS_INFO("Task Builder Node is up and running!!!");
	ros::Rate loop_rate(10.0);
  int task_id = 0;
  std::string task_name = "Trazer um Documento";
  std::string task_description = "Ir até a mesa do Chistiano, pegar o RG dele e trazer de volta para mim.";
  std::vector<unifei::expertinos::mrta_vc::tasks::Skill> task_desired_skills;
  task_desired_skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("velocity", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
  task_desired_skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("strength", unifei::expertinos::mrta_vc::tasks::levels::HIGH));
  task_desired_skills.push_back(unifei::expertinos::mrta_vc::tasks::Skill("flexible", unifei::expertinos::mrta_vc::tasks::levels::LOW));
  mrta_vc::GetUser user_srv;
  user_srv.request.name = "Adriano Henrique Rossette Leite";
  if (!get_user_cli_.call(user_srv))
  {
    ROS_ERROR("There is no user register as %s!!!", user_srv.request.name.c_str());
    ROS_ERROR("%s", user_srv.response.message.c_str());
  }
  unifei::expertinos::mrta_vc::agents::User task_sender(user_srv.response.user);
  mrta_vc::GetPerson person_srv;
  user_srv.request.name = "Christiano Henrique Rezende";
  if (!get_person_cli_.call(person_srv))
  {
    ROS_ERROR("There is no person register as %s!!!", person_srv.request.name.c_str());
    ROS_ERROR("%s", person_srv.response.message.c_str());
  }
  unifei::expertinos::mrta_vc::agents::Person task_receiver(person_srv.response.person);
  unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum task_priority = unifei::expertinos::mrta_vc::tasks::priorities::IMPORTANT;
  ros::Time task_deadline = ros::Time::now() + ros::Duration(120);
  task_ = unifei::expertinos::mrta_vc::tasks::Task(task_id, task_name, task_description, task_desired_skills, task_sender, task_receiver, task_deadline, task_priority);
  //ROS_INFO("%s", task_.toString().c_str());
  task_pub_.publish(task_.toMsg());
	while (nh_.ok()) 
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/**
 * 
 */
bool mrta_vc::TaskBuilderNode::abort(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  return false;
}
