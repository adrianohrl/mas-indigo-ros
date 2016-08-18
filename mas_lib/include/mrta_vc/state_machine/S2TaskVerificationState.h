/**
 *  S2TaskVerificationState.h
 *	
 *  Corresponds to S5 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.2
 *  Created on: 12/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_S2_TASK_VERIFICATION_STATE_H_
#define TASK_BUILDER_S2_TASK_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/TaskVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
    class S2TaskVerificationState : public TaskVerificationState
		{

		public:
			S2TaskVerificationState(MachineController* controller);
			virtual ~S2TaskVerificationState();

			virtual bool process(std::string answer);
			virtual std::string toString();

    private:
			virtual bool next(std::string answer);

 		};
	}
}		
					
#endif /* TASK_BUILDER_S2_TASK_VERIFICATION_STATE_H_ */
