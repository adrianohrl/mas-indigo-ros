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
mrta_vc::SystemRobotInterfaceNode::SystemRobotInterfaceNode(ros::NodeHandle nh) : unifei::expertinos::mrta_vc::agents::Robot(), nh_(nh), execute_action_srv_(nh, "/execute_task", boost::bind(&mrta_vc::SystemRobotInterfaceNode::executeCallback, this, _1), false)
{
	beacon_timer_ = nh_.createTimer(ros::Duration(ROBOT_BEACON_INTERVAL_DURATION), &mrta_vc::SystemRobotInterfaceNode::beaconTimerCallback, this);
	task_end_timer_ = nh_.createTimer(ros::Duration(10), &mrta_vc::SystemRobotInterfaceNode::taskEndTimerCallback, this); // para testes
	beacon_pub_ = nh_.advertise<mrta_vc::Agent>("/robots", 1);
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
	beacon_pub_.shutdown();
	execute_action_srv_.shutdown();
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
 * This callback send a beacon signal of this robot to the system manager.
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
	//ROS_INFO("task end cb!!!");
	if (isBusy())
	{
		allocation_.end();
		ROS_INFO("Successfully ending %s task!", allocation_.getTask().getName().c_str());
	}
}

/**
 * This function set the current allocation to a final state.
 */
bool mrta_vc::SystemRobotInterfaceNode::finishAllocationCallback(mrta_vc::FinishAllocation::Request& request, mrta_vc::FinishAllocation::Response& response)
{
	unifei::expertinos::mrta_vc::tasks::AllocationStateEnum state = unifei::expertinos::mrta_vc::tasks::AllocationStates::toEnumerated(request.state);
	response.valid = allocation_.finish(state);
	response.message = "Invalid state code!!!";
	if (response.valid)
	{
		response.message = "Finished!!!";
	}
	return response.valid;
}

/**
 * This callback receives new allocations from system manager.
 */
void mrta_vc::SystemRobotInterfaceNode::executeCallback(const mrta_vc::ExecuteGoal::ConstPtr& goal)
{
	ros::Rate loop_rate(10);
	unifei::expertinos::mrta_vc::tasks::Allocation allocation(goal->allocation);
	ROS_WARN("[GOAL CB] Got new allocation: %d, and this: %d", allocation.getTask().getId(), allocation_.getTask().getId());
	if (!allocation.isInvolved(*this))
	{
		ROS_WARN("Not involved!!!");
		return;
	}
	allocation_ = allocation;
	allocation_.start();
	while (nh_.ok())
	{
		if (execute_action_srv_.isPreemptRequested())
		{
			if (execute_action_srv_.isNewGoalAvailable())
			{
				allocation = unifei::expertinos::mrta_vc::tasks::Allocation(execute_action_srv_.acceptNewGoal()->allocation);
				if (allocation == allocation_)
				{
					ROS_WARN("You've already sent this allocation before!!!");
					continue;
				}
				ROS_WARN("[GOAL CB] Got new allocation while executing one: %d, and this: %d", allocation.getTask().getId(), allocation_.getTask().getId());
				if (!allocation.isInvolved(*this))
				{
					ROS_WARN("Not accepted!!!");
					ROS_ERROR("What do I do here????");
					return;
				}
			}
			ROS_INFO("Canceling!!!");
			allocation_.cancel();
			publishExecuteResult();
			return;
		}
		executingTask();
		publishExecuteFeedback();
		if (allocation_.isFinished())
		{
			publishExecuteResult();
			ROS_INFO("Sending result!!!");
			return;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("Aborting allocation: this node has been killed!!!");
	allocation_.abort();
	publishExecuteResult();
}

/**
 *
 */
void mrta_vc::SystemRobotInterfaceNode::executingTask()
{

}

/**
 * This function prepares the execute action feedback message to send to client
 */
void mrta_vc::SystemRobotInterfaceNode::publishExecuteFeedback()
{
	if (allocation_.isFinished())
	{
		return;
	}
	mrta_vc::ExecuteFeedback feedback;
	feedback.allocation = allocation_.toMsg();
	execute_action_srv_.publishFeedback(feedback);
}

/**
 * This function prepares the execute action result message to send to client, and also prepares
 * this server to receive a new goal allocation
 */
void mrta_vc::SystemRobotInterfaceNode::publishExecuteResult(std::string message)
{	
	mrta_vc::ExecuteResult result;
	result.allocation = allocation_.toMsg();
	result.message = message;
	switch (allocation_.getState())
	{
		case unifei::expertinos::mrta_vc::tasks::states::ABORTED:
			execute_action_srv_.setAborted(result, message);
			break;
		case unifei::expertinos::mrta_vc::tasks::states::CANCELLED:
			execute_action_srv_.setPreempted(result, message);
			break;
		case unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED:
			execute_action_srv_.setSucceeded(result, message);
			break;
		default:
			ROS_ERROR("Unable to publish execute action result: NOT A FINAL STATE!!!");
			ROS_ERROR("state: %s", unifei::expertinos::mrta_vc::tasks::AllocationStates::toString(allocation_.getState()).c_str());
			return;
	}
	allocation_ = unifei::expertinos::mrta_vc::tasks::Allocation();
}

/**
 * This methods sets this object to a robot according to the giving YAML file when this application
 * started.
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
		execute_action_srv_.start();
	}
	else
	{
		ROS_ERROR("You must create and set a YAML file containing this robot info!!!");
	}
	setted_up_ = setted_up;
}

/**
 * Checks if this robot is not executing any task.
 */
bool mrta_vc::SystemRobotInterfaceNode::isAvailable()
{
	return allocation_.getTask().getName() == "";
}

/**
 * Check if this robot is executing a task.
 */
bool mrta_vc::SystemRobotInterfaceNode::isBusy()
{
	return !isAvailable();
}
