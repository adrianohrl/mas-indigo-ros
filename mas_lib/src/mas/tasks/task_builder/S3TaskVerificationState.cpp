/**
 *  S3TaskVerificationState.cpp
 *
 *	Corresponds to S3 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/S3TaskVerificationState.h"
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
			S3TaskVerificationState::S3TaskVerificationState(MachineController* controller) : TaskVerificationState(controller)
			{
			}

			/**
			 * Destructor
			 */
			S3TaskVerificationState::~S3TaskVerificationState()
			{
			}

			/**
			 *
			 */
			bool S3TaskVerificationState::process(std::string answer)
			{
				answer = "take " + answer;
				if (TaskVerificationState::process(answer))
				{
					return next(answer);
				}
				return false;
			}

			/**
			 *
			 */
			bool S3TaskVerificationState::next(std::string answer)
			{
					AbstractState::getController()->setNextToS5();
					return true;
			}

			/**
			 *
			 */
			std::string S3TaskVerificationState::toString()
			{
				return "S3 (Task Verification State)";
			}
			
		}
	}
}
