/**
 *  S1TaskVerificationState.cpp
 *
 *	Corresponds to S1 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/S1TaskVerificationState.h"
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
			S1TaskVerificationState::S1TaskVerificationState(MachineController* controller) : TaskVerificationState(controller)
			{
			}

			/**
			 * Destructor
			 */
			S1TaskVerificationState::~S1TaskVerificationState()
			{
			}

			/**
			 *
			 */
			bool S1TaskVerificationState::process(std::string answer)
			{
				answer = "bring " + answer;
				if (TaskVerificationState::process(answer))
				{
					AbstractState::getController()->setTaskReceiver(AbstractState::getController()->getUser());
					return next(answer);
				}
				return false;
			}

			/**
			 *
			 */
			bool S1TaskVerificationState::next(std::string answer)
			{
					AbstractState::getController()->setNextToS4();
					return true;
			}

			/**
			 *
			 */
			std::string S1TaskVerificationState::toString()
			{
				return "S1 (Task Verification State)";
			}
			
		}
	}
}
