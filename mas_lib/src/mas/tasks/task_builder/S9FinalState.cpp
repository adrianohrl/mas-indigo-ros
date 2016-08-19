/**
 *  S9FinalState.cpp
 *
 *	Corresponds to S9 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 15/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/S9FinalState.h"
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
			S9FinalState::S9FinalState(MachineController* controller) : AbstractState(controller, "", true)
			{
			}

			/**
			 * Destructor
			 */
			S9FinalState::~S9FinalState()
			{
			}

			/**
			 *
			 */
			bool S9FinalState::process(std::string answer)
			{
				return next(answer);
			}

			/**
			 *
			 */
			bool S9FinalState::next(std::string answer)
			{
				AbstractState::getController()->setNextToS0();
				return true;
			}

			/**
			 *
			 */
			std::string S9FinalState::toString()
			{
				return "S9 (Final State)";
			}
			
		}
	}
}
