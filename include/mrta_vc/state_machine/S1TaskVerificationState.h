/**
 *  S1TaskVerificationState.h
 *	
 *  Corresponds to S5 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef S1_TASK_VERIFICATION_STATE_H_
#define S1_TASK_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/TaskVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
    class S1TaskVerificationState : public TaskVerificationState
		{

		public:
			S1TaskVerificationState(MachineController* controller);
			~S1TaskVerificationState();

    private:
      virtual void next();

 		};
	}
}		
					
#endif /* S1_TASK_VERIFICATION_STATE_H_ */
