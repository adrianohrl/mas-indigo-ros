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
	task_end_timer_ = nh_.createTimer(ros::Duration(10), &mrta_vc::SystemRobotInterfaceNode::taskEndTimerCallback, this); // para testes
	allocation_timer_ = nh_.createTimer(ros::Duration(ALLOCATION_INTERVAL_DURATION), &mrta_vc::SystemRobotInterfaceNode::allocationTimerCallback, this);
  beacon_pub_ = nh_.advertise<mrta_vc::Agent>("/robots", 1);
	allocation_pub_ = nh_.advertise<mrta_vc::Allocation>("/running_allocations", 1);
	allocation_sub_ = nh_.subscribe("/allocations", 1, &mrta_vc::SystemRobotInterfaceNode::allocationsCallback, this);
	allocation_cancellation_sub_ = nh_.subscribe("/allocation_cancellations", 1, &mrta_vc::SystemRobotInterfaceNode::allocationCancellationsCallback, this);
	allocation_abortion_sub_ = nh_.subscribe("/allocation_abortions", 1, &mrta_vc::SystemRobotInterfaceNode::allocationAbortionsCallback, this);
  setted_up_ = false;
  setUp();
}

/**
 * Destructor
 */
mrta_vc::SystemRobotInterfaceNode::~SystemRobotInterfaceNode()
{
	beacon_timer_.stop();
	task_end_timer_.stop(); // para testes
	allocation_timer_.stop();
	beacon_pub_.shutdown();
	allocation_pub_.shutdown();
	allocation_sub_.shutdown();
	allocation_cancellation_sub_.shutdown();
	allocation_abortion_sub_.shutdown();
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
 * PARA TESTES
 */
void mrta_vc::SystemRobotInterfaceNode::taskEndTimerCallback(const ros::TimerEvent& event)
{
	if (isBusy())
	{
		allocation_.end();
		allocation_pub_.publish(allocation_.toMsg());
		allocation_ = unifei::expertinos::mrta_vc::tasks::Allocation();
	}
}

/**
 *
 */
void mrta_vc::SystemRobotInterfaceNode::allocationTimerCallback(const ros::TimerEvent& event)
{
	if (allocation_.isExecuting())
	{
		allocation_pub_.publish(allocation_.toMsg());
	}
	if (allocation_.isFinished())
	{
		allocation_pub_.publish(allocation_.toMsg());
		allocation_ = unifei::expertinos::mrta_vc::tasks::Allocation();
	}
}

/**
 *
 */
void mrta_vc::SystemRobotInterfaceNode::allocationsCallback(const mrta_vc::Allocation::ConstPtr& allocation_msg)
{
	if (isAvailable())
	{
		unifei::expertinos::mrta_vc::tasks::Allocation allocation(allocation_msg);
		if (allocation.isInvolved(*this))
		{
			allocation_ = allocation;
			allocation_.start();
			allocation_pub_.publish(allocation_.toMsg());
		}
	}
}

/**
 *
 */
void mrta_vc::SystemRobotInterfaceNode::allocationCancellationsCallback(const mrta_vc::Task::ConstPtr& task_msg)
{
	if (isBusy())
	{
		unifei::expertinos::mrta_vc::tasks::Allocation allocation;
		allocation = unifei::expertinos::mrta_vc::tasks::Allocation(unifei::expertinos::mrta_vc::tasks::Task(task_msg));
		if (allocation_ == allocation)
		{
			allocation_.cancel();
			allocation_pub_.publish(allocation_.toMsg());
		}
	}
}

/**
 *
 */
void mrta_vc::SystemRobotInterfaceNode::allocationAbortionsCallback(const mrta_vc::Task::ConstPtr& task_msg)
{
	if (isBusy())
	{
		unifei::expertinos::mrta_vc::tasks::Allocation allocation;
		allocation = unifei::expertinos::mrta_vc::tasks::Allocation(unifei::expertinos::mrta_vc::tasks::Task(task_msg));
		if (allocation_ == allocation)
		{
			allocation_.abort();
			allocation_pub_.publish(allocation_.toMsg());
		}
	}
}

/**
 *
 */
bool mrta_vc::SystemRobotInterfaceNode::finishAllocationCallback(mrta_vc::FinishAllocation::Request& request, mrta_vc::FinishAllocation::Response& response)
{
	unifei::expertinos::mrta_vc::tasks::TaskStateEnum state = unifei::expertinos::mrta_vc::tasks::TaskStates::toEnumerated(request.state);
	response.valid = allocation_.finish(state);
	response.message = "Invalid state code!!!";
	if (response.valid)
	{
		response.message = "Finished!!!";
	}
	return response.valid;
}

/**
 *
 */
void mrta_vc::SystemRobotInterfaceNode::setUp()
{
	bool setted_up = false;
  ROS_DEBUG("********* Reading Robot Parameters **********");
  std::string ns = ros::this_node::getName();

  std::string hostname;
  nh_.param<std::string>(ns + std::string("/hostname"), hostname, "");
  setted_up = hostname != "";
  ROS_ERROR_COND(!setted_up, "Invalid robot hostname!!!");

  bool mobile, holonomic;
  nh_.param<bool>(ns + std::string("/mobile"), mobile, false);
  nh_.param<bool>(ns + std::string("/holonomic"), holonomic, false);

  double location_x, location_y, location_theta;
  nh_.param<double>(ns + std::string("/location/x"), location_x, 0);
  nh_.param<double>(ns + std::string("/location/y"), location_y, 0);
  nh_.param<double>(ns + std::string("/location/theta"), location_theta, 0);

  unifei::expertinos::mrta_vc::agents::Robot robot(0, hostname, holonomic, mobile, location_x, location_y, location_theta);
  unifei::expertinos::mrta_vc::agents::Robot::operator=(robot);

  ns.append("/skill");
  int counter = 0;
  std::stringstream aux;
  aux << ns << counter;
  while (nh_.hasParam(aux.str() + "/resource/name"))
  {
    std::string resource_name, resource_description, level_name;
    nh_.param<std::string>(aux.str() + "/resource/name", resource_name, "");
    nh_.param<std::string>(aux.str() + "/resource/description", resource_description, "");
    nh_.param<std::string>(aux.str() + "/level", level_name, "");
    unifei::expertinos::mrta_vc::tasks::Resource resource(0, resource_name, resource_description);
    unifei::expertinos::mrta_vc::tasks::SkillLevelEnum level = unifei::expertinos::mrta_vc::tasks::SkillLevels::toEnumerated(level_name);
    unifei::expertinos::mrta_vc::tasks::Skill skill(0, resource, level);
    unifei::expertinos::mrta_vc::agents::Robot::addSkill(skill);
    aux.str("");
    aux << ns << ++counter;
  }
	if (setted_up)
  {
		ROS_INFO("This robot has been setted up!!!");
		ROS_INFO("Robot Info:");
		ROS_INFO("     hostname: %s", unifei::expertinos::mrta_vc::agents::Robot::getHostname().c_str());
		ROS_INFO("     holonomic: %s", unifei::expertinos::mrta_vc::agents::Robot::isHolonomic() ? "true" : "false");
		ROS_INFO("     mobile: %s", unifei::expertinos::mrta_vc::agents::Robot::isMobile() ? "true" : "false");
		ROS_INFO("     location: (%f, %f, %f)", unifei::expertinos::mrta_vc::agents::Robot::getLocation().getX(), unifei::expertinos::mrta_vc::agents::Robot::getLocation().getY(), unifei::expertinos::mrta_vc::agents::Robot::getLocation().getTheta());
		for (int i = 0; i < unifei::expertinos::mrta_vc::agents::Robot::getSkills().size(); i++)
		{
			unifei::expertinos::mrta_vc::tasks::Skill skill(unifei::expertinos::mrta_vc::agents::Robot::getSkills().at(i));
			ROS_INFO("     skill %d:", i);
			ROS_INFO("          resource: %s", skill.getResource().getName().c_str());
			ROS_INFO("          level: %s", unifei::expertinos::mrta_vc::tasks::SkillLevels::toString(skill.getLevel()).c_str());
		}
  }
  else
  {
		ROS_ERROR("You must create and set a YAML file containing this robot info!!!");
  }
	setted_up_ = setted_up;
}

/**
 *
 */
bool mrta_vc::SystemRobotInterfaceNode::isAvailable()
{
	return allocation_.getTask().getName() == "";
}

/**
 *
 */
bool mrta_vc::SystemRobotInterfaceNode::isBusy()
{
	return !isAvailable();
}
