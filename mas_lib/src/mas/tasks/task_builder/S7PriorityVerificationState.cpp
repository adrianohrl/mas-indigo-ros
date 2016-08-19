/**
 *  S7PriorityVerificationState.cpp
 *
 *	Corresponds to S7 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/S7PriorityVerificationState.h"
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
			S7PriorityVerificationState::S7PriorityVerificationState(MachineController* controller) : AbstractState(controller, "How urgent?")
			{
			}

			/**
			 * Destructor
			 */
			S7PriorityVerificationState::~S7PriorityVerificationState()
			{
			}

			/**
			 *
			 */
			bool S7PriorityVerificationState::process(std::string answer)
			{
			  if (tasks::TaskPriorities::isValid(answer))
			  {
					AbstractState::getController()->setTaskPriority(tasks::TaskPriorities::toEnumerated(answer));
					return next(answer);
			  }
				return false;
			}

			/**
			 *
			 */
			bool S7PriorityVerificationState::next(std::string answer)
			{
				AbstractState::getController()->setNextToS8();
				return true;
			}

			/**
			 *
			 */
			std::string S7PriorityVerificationState::toString()
			{
				return "S7 (Priority Verification State)";
			}
			
		}
	}
}
