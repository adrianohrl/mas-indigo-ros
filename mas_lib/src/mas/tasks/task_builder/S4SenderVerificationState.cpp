/**
 *  S4SenderVerificationState.cpp
 *
 *	Corresponds to S4 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/S4SenderVerificationState.h"
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
			S4SenderVerificationState::S4SenderVerificationState(MachineController* controller) : SenderVerificationState(controller)
			{
			}

			/**
			 * Destructor
			 */
			S4SenderVerificationState::~S4SenderVerificationState()
			{
			}

			/**
			 *
			 */
			bool S4SenderVerificationState::process(std::string answer)
			{
				if (SenderVerificationState::process(answer))
				{
					return next(answer);
				}
				return false;
			}

			/**
			 *
			 */
			bool S4SenderVerificationState::next(std::string answer)
			{
				AbstractState::getController()->setNextToS7();
				return true;
			}

			/**
			 *
			 */
			std::string S4SenderVerificationState::toString()
			{
				return "S4 (Sender Verification State)";
			}
			
		}
	}
}
