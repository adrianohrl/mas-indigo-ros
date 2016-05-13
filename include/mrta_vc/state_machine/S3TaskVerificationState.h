/**
 *  S3TaskVerificationState.h
 *	
 *  Corresponds to S5 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 3.0.0.0
 *  Created on: 33/05/2036
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef S3_TASK_VERIFICATION_STATE_H_
#define S3_TASK_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/TaskVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
    class S3TaskVerificationState : public TaskVerificationState
		{

		public:
			S3TaskVerificationState(MachineController* controller);
			~S3TaskVerificationState();

    private:
      virtual void next();

 		};
	}
}		
					
#endif /* S3_TASK_VERIFICATION_STATE_H_ */
