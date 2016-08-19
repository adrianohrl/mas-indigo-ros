/**
 *  S2TaskVerificationState.cpp
 *
 *	Corresponds to S2 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/S2TaskVerificationState.h"
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
			S2TaskVerificationState::S2TaskVerificationState(MachineController* controller) : TaskVerificationState(controller)
			{
			}

			/**
			 * Destructor
			 */
			S2TaskVerificationState::~S2TaskVerificationState()
			{
			}

			/**
			 *
			 */
			bool S2TaskVerificationState::process(std::string answer)
			{
				answer = "send " + answer;
				if (TaskVerificationState::process(answer))
				{
					AbstractState::getController()->setTaskSender(AbstractState::getController()->getUser());
					return next(answer);
				}
				return false;
			}

			/**
			 *
			 */
			bool S2TaskVerificationState::next(std::string answer)
			{
					AbstractState::getController()->setNextToS6();
					return true;
			}

			/**
			 *
			 */
			std::string S2TaskVerificationState::toString()
			{
				return "S2 (Task Verification State)";
			}
			
		}
	}
}
