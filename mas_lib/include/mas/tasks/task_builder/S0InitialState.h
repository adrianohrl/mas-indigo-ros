/**
 *  S0InitialState.h
 *
 *  Corresponds to S0 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.4
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_S0_INITIAL_STATE_H_
#define TASK_BUILDER_S0_INITIAL_STATE_H_

#include "mas/tasks/task_builder/AbstractState.h"

namespace mas
{
	namespace tasks
	{
		namespace task_builder
		{
			class S0InitialState : public AbstractState
			{

			public:
				S0InitialState(MachineController* controller);
				virtual ~S0InitialState();

				virtual bool process(std::string answer);
				virtual std::string toString();

	    private:
				virtual bool next(std::string answer);

	 		};
		}
	}		
}
					
#endif /* TASK_BUILDER_S0_INITIAL_STATE_H_ */
