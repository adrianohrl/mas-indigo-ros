/**
 *  SenderVerificationState.cpp
 *
 *  Version: 1.2.4
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/SenderVerificationState.h"
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
			SenderVerificationState::SenderVerificationState(MachineController* controller) : PersonVerificationState(controller, "From whom?")
			{
			}

			/**
			 * Destructor
			 */
			SenderVerificationState::~SenderVerificationState()
			{
			}

			/**
			 *
			 */
			bool SenderVerificationState::process(std::string answer)
			{
				if (PersonVerificationState::process(answer))
				{
					AbstractState::getController()->setTaskSender(PersonVerificationState::getPerson());
					return true;
			  }
				return false;
			}

			/**
			 *
			 */
			bool SenderVerificationState::next(std::string answer)
			{
				return false;
			}

			/**
			 *
			 */
			std::string SenderVerificationState::toString()
			{
				return "";
			}
			
		}
	}
}
