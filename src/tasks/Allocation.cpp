/**
 *  Allocation.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 08/04/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/Allocation.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::Allocation()
{
	state_ = unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED;
	satisfaction_ = unifei::expertinos::mrta_vc::tasks::satisfactions::NONE;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::Allocation(unifei::expertinos::mrta_vc::tasks::Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots, unifei::expertinos::mrta_vc::tasks::AllocationStateEnum state, unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum satisfaction, ros::Time allocation_timestamp, ros::Time dispatch_timestamp, ros::Time start_timestamp, ros::Time end_timestamp) : task_(task), robots_(robots)
{
	state_ = state;
	satisfaction_ = satisfaction;
	state_ = unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED;
	allocation_timestamp_ = allocation_timestamp;
	setDispatchTimestamp(dispatch_timestamp);
	setStartTimestamp(start_timestamp);
	setEndTimestamp(end_timestamp);
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::Allocation(unifei::expertinos::mrta_vc::tasks::Task task, std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots) : task_(task)
{
	state_ = unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED;
	satisfaction_ = unifei::expertinos::mrta_vc::tasks::satisfactions::NONE;
	allocate(robots);
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::Allocation(const ::mrta_vc::Allocation::ConstPtr& allocation_msg) : task_(allocation_msg->task)
{
	for (int i = 0; i < allocation_msg->robots.size(); i++)
	{
		unifei::expertinos::mrta_vc::agents::Robot robot(allocation_msg->robots.at(i));
		robots_.push_back(robot);
	}
	state_ = AllocationStates::toEnumerated(allocation_msg->state);
	satisfaction_ = TaskSatisfactions::toEnumerated(allocation_msg->satisfaction);
	allocation_timestamp_ = allocation_msg->allocation_timestamp;
	setDispatchTimestamp(allocation_msg->dispatch_timestamp);
	setStartTimestamp(allocation_msg->start_timestamp);
	setEndTimestamp(allocation_msg->end_timestamp);
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::Allocation(::mrta_vc::Allocation allocation_msg) : task_(allocation_msg.task)
{
	for (int i = 0; i < allocation_msg.robots.size(); i++)
	{
		unifei::expertinos::mrta_vc::agents::Robot robot(allocation_msg.robots.at(i));
		robots_.push_back(robot);
	}
	state_ = AllocationStates::toEnumerated(allocation_msg.state);
	satisfaction_ = TaskSatisfactions::toEnumerated(allocation_msg.satisfaction);
	allocation_timestamp_ = allocation_msg.allocation_timestamp;
	setDispatchTimestamp(allocation_msg.dispatch_timestamp);
	setStartTimestamp(allocation_msg.start_timestamp);
	setEndTimestamp(allocation_msg.end_timestamp);	
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Allocation::~Allocation() 
{
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Task unifei::expertinos::mrta_vc::tasks::Allocation::getTask() 
{
	return task_;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::agents::Robot> unifei::expertinos::mrta_vc::tasks::Allocation::getRobots() 
{
	return robots_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::AllocationStateEnum unifei::expertinos::mrta_vc::tasks::Allocation::getState() 
{
	return state_;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum unifei::expertinos::mrta_vc::tasks::Allocation::getSatisfaction() 
{
	return satisfaction_;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::tasks::Allocation::getAllocationTimestamp()
{
	return allocation_timestamp_;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::tasks::Allocation::getDispatchTimestamp()
{
	return dispatch_timestamp_;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::tasks::Allocation::getStartTimestamp() 
{
	return start_timestamp_;
}

/**
 *
 */
ros::Time unifei::expertinos::mrta_vc::tasks::Allocation::getEndTimestamp() 
{
	return end_timestamp_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasAllocated()
{
	return state_ != unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasDispatched()
{
	return wasAllocated() && state_ != unifei::expertinos::mrta_vc::tasks::states::ALLOCATED;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasAccepted() 
{
	return wasDispatched() && state_ != unifei::expertinos::mrta_vc::tasks::states::DISPATCHED;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::isExecuting()
{
	return state_ == unifei::expertinos::mrta_vc::tasks::states::EXECUTING;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::isFinished()
{
	return state_ == unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED ||
			state_ == unifei::expertinos::mrta_vc::tasks::states::ABORTED ||
			state_ == unifei::expertinos::mrta_vc::tasks::states::CANCELLED;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasSucceeded() 
{
	return state_ == unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasAborted() 
{
	return state_ == unifei::expertinos::mrta_vc::tasks::states::ABORTED;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasCancelled() 
{
	return state_ == unifei::expertinos::mrta_vc::tasks::states::CANCELLED;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::wasEvaluated()
{
	return satisfaction_ != unifei::expertinos::mrta_vc::tasks::satisfactions::NONE;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::addRobots(std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots)
{
	for (int i = 0; i < robots.size(); i++)
	{
		addRobot(robots.at(i));
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::addRobot(unifei::expertinos::mrta_vc::agents::Robot robot) 
{
	for (int i = 0; i < robots_.size(); i++)
	{
		if(robot.equals(robots_.at(i)))
		{
			return;
		}
	}	
	robots_.push_back(robot);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::removeRobot(unifei::expertinos::mrta_vc::agents::Robot robot) 
{
	for (int i = 0; i < robots_.size(); i++)
	{
		if(robot.equals(robots_.at(i)))
		{
			robots_.erase(robots_.begin() + i);
			return;
		}
	}
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::isValid(unifei::expertinos::mrta_vc::tasks::AllocationStateEnum state)
{
	return state_ != unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED &&
				 (state == unifei::expertinos::mrta_vc::tasks::states::ABORTED || state == unifei::expertinos::mrta_vc::tasks::states::CANCELLED ||
					state_ != unifei::expertinos::mrta_vc::tasks::states::ABORTED && state_ != unifei::expertinos::mrta_vc::tasks::states::CANCELLED &&
					(state_ == unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED && state == unifei::expertinos::mrta_vc::tasks::states::ALLOCATED ||
					 state_ == unifei::expertinos::mrta_vc::tasks::states::ALLOCATED && state == unifei::expertinos::mrta_vc::tasks::states::DISPATCHED ||
					 state_ == unifei::expertinos::mrta_vc::tasks::states::DISPATCHED && state == unifei::expertinos::mrta_vc::tasks::states::EXECUTING ||
					 state_ == unifei::expertinos::mrta_vc::tasks::states::EXECUTING && state == unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED));
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setState(unifei::expertinos::mrta_vc::tasks::AllocationStateEnum state)
{
	if (isValid(state))
	{
		ROS_ERROR("[SET STATE] state: %s", unifei::expertinos::mrta_vc::tasks::AllocationStates::toString(state).c_str());
		state_ = state;
	}
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::hasStateChanged(unifei::expertinos::mrta_vc::tasks::AllocationStateEnum state)
{
	state_ != state;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setSatisfaction(unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum satisfaction)
{
	satisfaction_ = satisfaction;
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setAllocationTimestamp(ros::Time allocation_timestamp)
{
	if (!allocation_timestamp_.isValid())
	{
		allocation_timestamp_ = allocation_timestamp;
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setDispatchTimestamp(ros::Time dispatch_timestamp)
{
	if (allocation_timestamp_.isValid() && !dispatch_timestamp_.isValid() && allocation_timestamp_ < dispatch_timestamp)
	{
		dispatch_timestamp_ = dispatch_timestamp;
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setStartTimestamp(ros::Time start_timestamp)
{
	if (dispatch_timestamp_.isValid() && !start_timestamp_.isValid() && dispatch_timestamp_ < start_timestamp)
	{
		start_timestamp_ = start_timestamp;
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::setEndTimestamp(ros::Time end_timestamp)
{
	if (start_timestamp_.isValid() && !end_timestamp_.isValid() && start_timestamp_ < end_timestamp)
	{
		end_timestamp_ = end_timestamp;
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::allocate(std::vector<unifei::expertinos::mrta_vc::agents::Robot> robots)
{
	if (!robots_.empty())
	{
		return;
	}
	ROS_ERROR("[ALLOCATION] Trying to allocate!!!");
	addRobots(robots);
	setState(unifei::expertinos::mrta_vc::tasks::states::ALLOCATED);
	ROS_INFO("[ALLOCATION] state: %s", unifei::expertinos::mrta_vc::tasks::AllocationStates::toString(state_).c_str());
	if (hasStateChanged(unifei::expertinos::mrta_vc::tasks::states::ALLOCATED))
	{
		setAllocationTimestamp();
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::dispatch()
{
	ROS_ERROR("[ALLOCATION] Trying to dispatch!!!");
	setState(unifei::expertinos::mrta_vc::tasks::states::DISPATCHED);
	if (hasStateChanged(unifei::expertinos::mrta_vc::tasks::states::DISPATCHED))
	{
		setAllocationTimestamp();
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::start()
{
	ROS_ERROR("[ALLOCATION] Trying to start!!!");
	setState(unifei::expertinos::mrta_vc::tasks::states::EXECUTING);
	if (hasStateChanged(unifei::expertinos::mrta_vc::tasks::states::EXECUTING))
	{
		setStartTimestamp();
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::end()
{
	setState(unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED);
	if (hasStateChanged(unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED))
	{
		setEndTimestamp();
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::abort()
{
	setState(unifei::expertinos::mrta_vc::tasks::states::ABORTED);
	if (hasStateChanged(unifei::expertinos::mrta_vc::tasks::states::ABORTED))
	{
		setEndTimestamp();
	}
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::cancel()
{
	setState(unifei::expertinos::mrta_vc::tasks::states::CANCELLED);
	if (hasStateChanged(unifei::expertinos::mrta_vc::tasks::states::CANCELLED))
	{
		setEndTimestamp();
	}
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::finish(unifei::expertinos::mrta_vc::tasks::states::AllocationStateEnum state)
{
	if (!isValid(state))
	{
		return false;
	}
	switch (state)
	{
		case unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED:
			end();
			break;
		case unifei::expertinos::mrta_vc::tasks::states::ABORTED:
			abort();
			break;
		case unifei::expertinos::mrta_vc::tasks::states::CANCELLED:
			cancel();
			break;
		default:
			return false;
	}
	return true;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::isInvolved(unifei::expertinos::mrta_vc::agents::Robot robot)
{
	for (int i = 0; i <robots_.size(); i++)
	{
		if (robot == robots_.at(i))
		{
			return true;
		}
	}
	return false;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::isInvolved(unifei::expertinos::mrta_vc::agents::Person person)
{
	return task_.isInvolved(person);
}

/**
 *
 */
::mrta_vc::Allocation unifei::expertinos::mrta_vc::tasks::Allocation::toMsg()
{
	::mrta_vc::Allocation allocation_msg;
	allocation_msg.task = task_.toMsg();
	for(int i = 0; i < robots_.size(); i++)
	{
		allocation_msg.robots.push_back(robots_.at(i).toMsg());
	}
	allocation_msg.state = unifei::expertinos::mrta_vc::tasks::AllocationStates::toCode(state_);
	allocation_msg.satisfaction = unifei::expertinos::mrta_vc::tasks::TaskSatisfactions::toCode(satisfaction_);
	allocation_msg.allocation_timestamp = allocation_timestamp_;
	allocation_msg.dispatch_timestamp = dispatch_timestamp_;
	allocation_msg.start_timestamp = start_timestamp_;
	allocation_msg.end_timestamp = end_timestamp_;
	return allocation_msg;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::tasks::Allocation::toString()
{
	std::stringstream robots_ss;
	for (int i = 0; i < robots_.size(); i++)
	{
		if (i != 0)
		{
			robots_ss << ", ";
		}
		robots_ss << i << " " << robots_.at(i).toString();
	}
	return "allocation: {" + task_.toString() +
			", robots: {" + robots_ss.str() +
			"}, state: " + unifei::expertinos::mrta_vc::tasks::AllocationStates::toString(state_) +
			", satisfaction: " + unifei::expertinos::mrta_vc::tasks::TaskSatisfactions::toString(satisfaction_) +
			", allocation timestamp: " + unifei::expertinos::utilities::TimeManipulator::toString(allocation_timestamp_) +
			", dispatch timestamp: " + unifei::expertinos::utilities::TimeManipulator::toString(dispatch_timestamp_) +
			", start timestamp: " + unifei::expertinos::utilities::TimeManipulator::toString(start_timestamp_) +
			", end timestamp: " + unifei::expertinos::utilities::TimeManipulator::toString(end_timestamp_) +
			"}";
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::Allocation::compareTo(unifei::expertinos::mrta_vc::tasks::Allocation allocation)
{
	return task_.compareTo(allocation.task_);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::equals(const unifei::expertinos::mrta_vc::tasks::Allocation& allocation)
{
	return operator==(allocation);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::operator==(const unifei::expertinos::mrta_vc::tasks::Allocation& allocation)
{
	return task_ == allocation.task_;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::Allocation::operator!=(const unifei::expertinos::mrta_vc::tasks::Allocation& allocation)
{
	return !operator==(allocation);
}

/**
 *
 */
void unifei::expertinos::mrta_vc::tasks::Allocation::operator=(const unifei::expertinos::mrta_vc::tasks::Allocation& allocation)
{
	task_ = allocation.task_;
	robots_ = allocation.robots_;
	state_ = allocation.state_;
	satisfaction_ = allocation.satisfaction_;
	allocation_timestamp_ = allocation.allocation_timestamp_;
	dispatch_timestamp_ = allocation.dispatch_timestamp_;
	start_timestamp_ = allocation.start_timestamp_;
	end_timestamp_ = allocation.end_timestamp_;
}
