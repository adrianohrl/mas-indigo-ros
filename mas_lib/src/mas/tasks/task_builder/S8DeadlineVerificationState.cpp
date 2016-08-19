/**
 *  S8DeadlineVerificationState.cpp
 *
 *	Corresponds to S8 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 14/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/S8DeadlineVerificationState.h"
#include "mas/tasks/task_builder/MachineController.h"

namespace mas
{
	namespace tasks
	{
		namespace task_builder
		{

			/**
			 * Constructor
			 */
			S8DeadlineVerificationState::S8DeadlineVerificationState(MachineController* controller) : AbstractState(controller, "What is the deadline?")
			{
			}

			/**
			 * Destructor
			 */
			S8DeadlineVerificationState::~S8DeadlineVerificationState()
			{
			}

			/**
			 *
			 */
			bool S8DeadlineVerificationState::process(std::string answer)
			{
				ros::Time deadline = utilities::TimeManipulator::getTime(answer);
				if (!utilities::TimeManipulator::isDeadline(deadline))
				{
					ros::Duration duration = utilities::TimeManipulator::getDuration(answer);
					if (!utilities::TimeManipulator::isDuration(duration))
					{
						return false;
					}
					deadline = utilities::TimeManipulator::getDeadline(duration);
				}
				AbstractState::getController()->setTaskDeadline(deadline);
				return next(answer);
			}

			/**
			 *
			 */
			bool S8DeadlineVerificationState::next(std::string answer)
			{
				AbstractState::getController()->setNextToS9();
				return true;
			}

			/**
			 *
			 */
			std::string S8DeadlineVerificationState::toString()
			{
				return "S8 (Deadline Verification State)";
			}
			
		}
	}
}
