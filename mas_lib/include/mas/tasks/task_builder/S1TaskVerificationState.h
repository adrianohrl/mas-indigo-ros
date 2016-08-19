/**
 *  S1TaskVerificationState.h
 *	
 *  Corresponds to S5 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_S1_TASK_VERIFICATION_STATE_H_
#define TASK_BUILDER_S1_TASK_VERIFICATION_STATE_H_

#include "mas/tasks/task_builder/TaskVerificationState.h"

namespace mas
{
	namespace tasks
	{
		namespace task_builder
		{
			class S1TaskVerificationState : public TaskVerificationState
			{

			public:
				S1TaskVerificationState(MachineController* controller);
				virtual ~S1TaskVerificationState();

				virtual bool process(std::string answer);
				virtual std::string toString();

			private:
				virtual bool next(std::string answer);

			};
		}
	}		
}
					
#endif /* TASK_BUILDER_S1_TASK_VERIFICATION_STATE_H_ */
