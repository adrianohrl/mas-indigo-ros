/**
 *  S5SenderVerificationState.cpp
 *
 *	Corresponds to S5 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/S5SenderVerificationState.h"
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
			S5SenderVerificationState::S5SenderVerificationState(MachineController* controller) : SenderVerificationState(controller)
			{
			}

			/**
			 * Destructor
			 */
			S5SenderVerificationState::~S5SenderVerificationState()
			{
			}

			/**
			 *
			 */
			bool S5SenderVerificationState::process(std::string answer)
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
			bool S5SenderVerificationState::next(std::string answer)
			{
				AbstractState::getController()->setNextToS6();
				return true;
			}

			/**
			 *
			 */
			std::string S5SenderVerificationState::toString()
			{
				return "S5 (Sender Verification State)";
			}
			
		}
	}
}
