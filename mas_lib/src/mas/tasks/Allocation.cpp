/**
 *  Allocation.cpp
 *
 *  Version: 1.2.4
 *  Created on: 08/04/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Heverton Machado Soares (sm.heverton@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/Allocation.h"

namespace mas
{
	namespace tasks
	{

		/**
		 *
		 */
		Allocation::Allocation()
		{
			state_ = states::NOT_ALLOCATED;
			satisfaction_ = satisfactions::NONE;
		}

		/**
		 *
		 */
		Allocation::Allocation(Task task, std::vector<agents::Robot> robots, AllocationStateEnum state, AllocationSatisfactionEnum satisfaction, ros::Time allocation_timestamp, ros::Time dispatch_timestamp, ros::Time start_timestamp, ros::Time end_timestamp) : task_(task), robots_(robots)
		{
			state_ = state;
			satisfaction_ = satisfaction;
			state_ = states::NOT_ALLOCATED;
			allocation_timestamp_ = allocation_timestamp;
			setDispatchTimestamp(dispatch_timestamp);
			setStartTimestamp(start_timestamp);
			setEndTimestamp(end_timestamp);
		}

		/**
		 *
		 */
		Allocation::Allocation(Task task, std::vector<agents::Robot> robots) : task_(task)
		{
			state_ = states::NOT_ALLOCATED;
			satisfaction_ = satisfactions::NONE;
			allocate(robots);
		}

		/**
		 *
		 */
		Allocation::Allocation(const mas_msgs::Allocation::ConstPtr& allocation_msg) : task_(allocation_msg->task)
		{
			for (int i = 0; i < allocation_msg->robots.size(); i++)
			{
				agents::Robot robot(allocation_msg->robots.at(i));
				robots_.push_back(robot);
			}
			state_ = AllocationStates::toEnumerated(allocation_msg->state);
			satisfaction_ = AllocationSatisfactions::toEnumerated(allocation_msg->satisfaction);
			allocation_timestamp_ = allocation_msg->allocation_timestamp;
			setDispatchTimestamp(allocation_msg->dispatch_timestamp);
			setStartTimestamp(allocation_msg->start_timestamp);
			setEndTimestamp(allocation_msg->end_timestamp);
		}

		/**
		 *
		 */
		Allocation::Allocation(mas_msgs::Allocation allocation_msg) : task_(allocation_msg.task)
		{
			for (int i = 0; i < allocation_msg.robots.size(); i++)
			{
				agents::Robot robot(allocation_msg.robots.at(i));
				robots_.push_back(robot);
			}
			state_ = AllocationStates::toEnumerated(allocation_msg.state);
			satisfaction_ = AllocationSatisfactions::toEnumerated(allocation_msg.satisfaction);
			allocation_timestamp_ = allocation_msg.allocation_timestamp;
			setDispatchTimestamp(allocation_msg.dispatch_timestamp);
			setStartTimestamp(allocation_msg.start_timestamp);
			setEndTimestamp(allocation_msg.end_timestamp);	
		}

		/**
		 *
		 */
		Allocation::~Allocation() 
		{
		}

		/**
		 *
		 */
		Task Allocation::getTask() 
		{
			return task_;
		}

		/**
		 *
		 */
		std::vector<agents::Robot> Allocation::getRobots() 
		{
			return robots_;
		}

		/**
		 *
		 */
		AllocationStateEnum Allocation::getState() 
		{
			return state_;
		}

		/**
		 *
		 */
		AllocationSatisfactionEnum Allocation::getSatisfaction() 
		{
			return satisfaction_;
		}

		/**
		 *
		 */
		ros::Time Allocation::getAllocationTimestamp()
		{
			return allocation_timestamp_;
		}

		/**
		 *
		 */
		ros::Time Allocation::getDispatchTimestamp()
		{
			return dispatch_timestamp_;
		}

		/**
		 *
		 */
		ros::Time Allocation::getStartTimestamp() 
		{
			return start_timestamp_;
		}

		/**
		 *
		 */
		ros::Time Allocation::getEndTimestamp() 
		{
			return end_timestamp_;
		}

		/**
		 *
		 */
		bool Allocation::wasAllocated()
		{
			return state_ != states::NOT_ALLOCATED;
		}

		/**
		 *
		 */
		bool Allocation::wasDispatched()
		{
			return wasAllocated() && state_ != states::ALLOCATED;
		}

		/**
		 *
		 */
		bool Allocation::wasAccepted() 
		{
			return wasDispatched() && state_ != states::DISPATCHED;
		}

		/**
		 *
		 */
		bool Allocation::isExecuting()
		{
			return state_ == states::EXECUTING;
		}

		/**
		 *
		 */
		bool Allocation::isFinished()
		{
			return state_ == states::SUCCEEDED ||
					state_ == states::ABORTED ||
					state_ == states::CANCELLED;
		}

		/**
		 *
		 */
		bool Allocation::wasSucceeded() 
		{
			return state_ == states::SUCCEEDED;
		}

		/**
		 *
		 */
		bool Allocation::wasAborted() 
		{
			return state_ == states::ABORTED;
		}

		/**
		 *
		 */
		bool Allocation::wasCancelled() 
		{
			return state_ == states::CANCELLED;
		}

		/**
		 *
		 */
		bool Allocation::wasEvaluated()
		{
			return isFinished();
		}

		/**
		 *
		 */
		void Allocation::addRobots(std::vector<agents::Robot> robots)
		{
			for (int i = 0; i < robots.size(); i++)
			{
				addRobot(robots.at(i));
			}
		}

		/**
		 *
		 */
		void Allocation::addRobot(agents::Robot robot) 
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
		void Allocation::removeRobot(agents::Robot robot) 
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
		bool Allocation::isValid(AllocationStateEnum state)
		{
			return state_ != states::SUCCEEDED &&
						 (state == states::ABORTED || state == states::CANCELLED ||
							state_ != states::ABORTED && state_ != states::CANCELLED &&
							(state_ == states::NOT_ALLOCATED && state == states::ALLOCATED ||
							 state_ == states::ALLOCATED && state == states::DISPATCHED ||
							 state_ == states::DISPATCHED && state == states::EXECUTING ||
							 state_ == states::EXECUTING && state == states::SUCCEEDED));
		}

		/**
		 *
		 */
		void Allocation::setState(AllocationStateEnum state)
		{
			if (isValid(state))
			{
				state_ = state;
			}
		}

		/**
		 *
		 */
		bool Allocation::hasStateChanged(AllocationStateEnum state)
		{
			state_ != state;
		}

		/**
		 *
		 */
		void Allocation::setSatisfaction(AllocationSatisfactionEnum satisfaction)
		{
			satisfaction_ = satisfaction;
		}

		/**
		 *
		 */
		void Allocation::setAllocationTimestamp(ros::Time allocation_timestamp)
		{
			if (!allocation_timestamp_.isValid())
			{
				allocation_timestamp_ = allocation_timestamp;
			}
		}

		/**
		 *
		 */
		void Allocation::setDispatchTimestamp(ros::Time dispatch_timestamp)
		{
			if (allocation_timestamp_.isValid() && !dispatch_timestamp_.isValid() && allocation_timestamp_ < dispatch_timestamp)
			{
				dispatch_timestamp_ = dispatch_timestamp;
			}
		}

		/**
		 *
		 */
		void Allocation::setStartTimestamp(ros::Time start_timestamp)
		{
			if (dispatch_timestamp_.isValid() && !start_timestamp_.isValid() && dispatch_timestamp_ < start_timestamp)
			{
				start_timestamp_ = start_timestamp;
			}
		}

		/**
		 *
		 */
		void Allocation::setEndTimestamp(ros::Time end_timestamp)
		{
			if (start_timestamp_.isValid() && !end_timestamp_.isValid() && start_timestamp_ < end_timestamp)
			{
				end_timestamp_ = end_timestamp;
			}
		}

		/**
		 *
		 */
		void Allocation::allocate(std::vector<agents::Robot> robots)
		{
			if (!robots_.empty())
			{
				return;
			}
			addRobots(robots);
			setState(states::ALLOCATED);
			if (hasStateChanged(states::ALLOCATED))
			{
				setAllocationTimestamp();
			}
		}

		/**
		 *
		 */
		void Allocation::dispatch()
		{
			setState(states::DISPATCHED);
			if (hasStateChanged(states::DISPATCHED))
			{
				setAllocationTimestamp();
			}
		}

		/**
		 *
		 */
		void Allocation::start()
		{
			setState(states::EXECUTING);
			if (hasStateChanged(states::EXECUTING))
			{
				setStartTimestamp();
			}
		}

		/**
		 *
		 */
		void Allocation::end()
		{
			setState(states::SUCCEEDED);
			if (hasStateChanged(states::SUCCEEDED))
			{
				setEndTimestamp();
			}
		}

		/**
		 *
		 */
		void Allocation::abort()
		{
			setState(states::ABORTED);
			if (hasStateChanged(states::ABORTED))
			{
				setEndTimestamp();
			}
		}

		/**
		 *
		 */
		void Allocation::cancel()
		{
			setState(states::CANCELLED);
			if (hasStateChanged(states::CANCELLED))
			{
				setEndTimestamp();
			}
		}

		/**
		 *
		 */
		bool Allocation::finish(states::AllocationStateEnum state)
		{
			if (!isValid(state))
			{
				return false;
			}
			switch (state)
			{
				case states::SUCCEEDED:
					end();
					break;
				case states::ABORTED:
					abort();
					break;
				case states::CANCELLED:
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
		bool Allocation::isInvolved(agents::Robot robot)
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
		bool Allocation::isInvolved(agents::Person person)
		{
			return task_.isInvolved(person);
		}

		/**
		 *
		 */
		mas_msgs::Allocation Allocation::toMsg()
		{
			mas_msgs::Allocation allocation_msg;
			allocation_msg.task = task_.toMsg();
			for(int i = 0; i < robots_.size(); i++)
			{
				allocation_msg.robots.push_back(robots_.at(i).toMsg());
			}
			allocation_msg.state = AllocationStates::toCode(state_);
			allocation_msg.satisfaction = AllocationSatisfactions::toCode(satisfaction_);
			allocation_msg.allocation_timestamp = allocation_timestamp_;
			allocation_msg.dispatch_timestamp = dispatch_timestamp_;
			allocation_msg.start_timestamp = start_timestamp_;
			allocation_msg.end_timestamp = end_timestamp_;
			return allocation_msg;
		}

		/**
		 *
		 */
		std::string Allocation::toString()
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
					"}, state: " + AllocationStates::toString(state_) +
					", satisfaction: " + AllocationSatisfactions::toString(satisfaction_) +
					", allocation timestamp: " + utilities::TimeManipulator::toString(allocation_timestamp_) +
					", dispatch timestamp: " + utilities::TimeManipulator::toString(dispatch_timestamp_) +
					", start timestamp: " + utilities::TimeManipulator::toString(start_timestamp_) +
					", end timestamp: " + utilities::TimeManipulator::toString(end_timestamp_) +
					"}";
		}

		/**
		 *
		 */
		int Allocation::compareTo(Allocation allocation)
		{
			return task_.compareTo(allocation.task_);
		}

		/**
		 *
		 */
		bool Allocation::equals(const Allocation& allocation)
		{
			return operator==(allocation);
		}

		/**
		 *
		 */
		bool Allocation::operator==(const Allocation& allocation)
		{
			return task_ == allocation.task_;
		}

		/**
		 *
		 */
		bool Allocation::operator!=(const Allocation& allocation)
		{
			return !operator==(allocation);
		}

		/**
		 *
		 */
		void Allocation::operator=(const Allocation& allocation)
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
		
	}
}
