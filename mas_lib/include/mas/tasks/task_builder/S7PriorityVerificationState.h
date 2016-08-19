/**
 *  S7PriorityVerificationState.h
 *
 *  Version: 1.2.4
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_S7_PRIORITY_VERIFICATION_STATE_H_
#define TASK_BUILDER_S7_PRIORITY_VERIFICATION_STATE_H_

#include "mas/tasks/task_builder/AbstractState.h"
#include "mas/tasks/TaskPriorities.h"

namespace mas
{
	namespace tasks
	{
		namespace task_builder
		{
			class S7PriorityVerificationState : public AbstractState
			{

			public:
				S7PriorityVerificationState(MachineController* controller);
				virtual ~S7PriorityVerificationState();

				virtual bool process(std::string answer);
				virtual std::string toString();

	 		private:
				virtual bool next(std::string answer);

			};
		}
	}	
}	
					
#endif /* TASK_BUILDER_S7_PRIORITY_VERIFICATION_STATE_H_ */
