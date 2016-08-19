/**
 *  S0InitialState.cpp
 *
 *	Corresponds to S0 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/S0InitialState.h"
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
			S0InitialState::S0InitialState(MachineController* controller) : AbstractState(controller, "What should I do?")
			{
			}

			/**
			 * Destructor
			 */
			S0InitialState::~S0InitialState()
			{
			}

			/**
			 * 
			 */
			bool S0InitialState::process(std::string answer)
			{ 
				return next(answer);
			}

			/**
			* 
			*/
			bool S0InitialState::next(std::string answer)
			{
				if (answer == "bring")
				{
					AbstractState::getController()->setNextToS1();
				}
				else if (answer == "send")
				{
					AbstractState::getController()->setNextToS2();
				}
				else if (answer == "take")
				{
					AbstractState::getController()->setNextToS3();
				}
				else
				{
					return false;
				}
				return true;
			}

			/**
			 *
			 */
			std::string S0InitialState::toString()
			{
				return "S0 (Initial State)";
			}
			
		}
	}
}
