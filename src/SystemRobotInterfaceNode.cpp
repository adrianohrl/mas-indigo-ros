/**
 *  SystemRobotInterfaceNode.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 26/03/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/SystemRobotInterfaceNode.h"

/**
 * Constructor
 */
mrta_vc::SystemRobotInterfaceNode::SystemRobotInterfaceNode(ros::NodeHandle nh) : unifei::expertinos::mrta_vc::agents::Robot(), nh_(nh)
{
	beacon_timer_ = nh_.createTimer(ros::Duration(ROBOT_BEACON_INTERVAL_DURATION), &mrta_vc::SystemRobotInterfaceNode::beaconTimerCallback, this);
	beacon_pub_ = nh_.advertise<mrta_vc::Agent>("/robot_beacon", 1);
  setted_up_ = false;

}

/**
 * Destructor
 */
mrta_vc::SystemRobotInterfaceNode::~SystemRobotInterfaceNode()
{
	beacon_pub_.shutdown();
}

/**
 * 
 */
void mrta_vc::SystemRobotInterfaceNode::spin() 
{
	ROS_INFO("System Robot Interface Node is up and running!!!");
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
void mrta_vc::SystemRobotInterfaceNode::beaconTimerCallback(const ros::TimerEvent& event)
{
  if (setted_up_)
  {
    beacon_pub_.publish(unifei::expertinos::mrta_vc::agents::Robot::toMsg());
  }
}

/**
 *
 */
void mrta_vc::SystemRobotInterfaceNode::setUp()
{
  bool setted_up;
  ROS_DEBUG("********* Reading Robot Parameters **********");
  std::string ns = ros::this_node::getName();

  std::string hostname;
  nh_.param<std::string>(ns + std::string("/hostname"), hostname, "");
  ROS_ERROR_COND(hostname == "", "Invalid robot hostname!!!");
  setted_up = hostname != "";

  bool mobile, holonomic;
  nh_.param<bool>(ns + std::string("/mobile"), mobile, false);
  nh_.param<bool>(ns + std::string("/holonomic"), holonomic, false);

  double location_x, location_y, location_theta;
  nh_.param<double>(ns + std::string("/location/x"), location_x, 0);
  nh_.param<double>(ns + std::string("/location/y"), location_y, 0);
  nh_.param<double>(ns + std::string("/location/theta"), location_theta, 0);

  ns.append("/skill");
  std::vector<unifei::expertinos::mrta_vc::tasks::Skill> skills;
  for (int counter = 0; nh_.hasParam(ns + std::string(counter + "/resource/name")); counter++)
  {
    std::string resource_name, resource_description, level_name;
    nh_.param<std::string>(ns + std::string(counter + "/resource/name"), resource_name, "");
    nh_.param<std::string>(ns + std::string(counter + "/resource/description"), resource_description, "");
    nh_.param<std::string>(ns + std::string(counter + "/level"), level_name, "");
    unifei::expertinos::mrta_vc::tasks::Resource resource(0, resource_name, resource_description);
    unifei::expertinos::mrta_vc::tasks::SkillLevelEnum level = unifei::expertinos::mrta_vc::tasks::SkillLevels::toEnumerated(level_name);
    unifei::expertinos::mrta_vc::tasks::Skill skill(0, resource, level);
    skills.push_back(skill);
  }
}
