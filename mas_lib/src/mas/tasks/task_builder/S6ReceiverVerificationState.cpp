/**
 *  S6ReceiverVerificationState.cpp
 *
 *	Corresponds to S6 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/S6ReceiverVerificationState.h"
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
			S6ReceiverVerificationState::S6ReceiverVerificationState(MachineController* controller) : PersonVerificationState(controller, "To whom?")
			{
			}

			/**
			 * Destructor
			 */
			S6ReceiverVerificationState::~S6ReceiverVerificationState()
			{
			}

			/**
			 * 
			 */
			bool S6ReceiverVerificationState::process(std::string answer)
			{ 
				if (PersonVerificationState::process(answer))
				{
					AbstractState::getController()->setTaskReceiver(PersonVerificationState::getPerson());
					return next(answer);
				}
				return false;
			}

			/**
			 *
			 */
			bool S6ReceiverVerificationState::next(std::string answer)
			{
				AbstractState::getController()->setNextToS7();
				return true;
			}

			/**
			 *
			 */
			std::string S6ReceiverVerificationState::toString()
			{
				return "S6 (Receiver Verification State)";
			}
			
		}
	}
}
