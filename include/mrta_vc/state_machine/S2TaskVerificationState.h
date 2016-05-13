/**
 *  S2TaskVerificationState.h
 *	
 *  Corresponds to S5 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 2.0.0.0
 *  Created on: 22/05/2026
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef S2_TASK_VERIFICATION_STATE_H_
#define S2_TASK_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/TaskVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
    class S2TaskVerificationState : public TaskVerificationState
		{

		public:
			S2TaskVerificationState(MachineController* controller);
			~S2TaskVerificationState();

    private:
      virtual void next();

 		};
	}
}		
					
#endif /* S2_TASK_VERIFICATION_STATE_H_ */
