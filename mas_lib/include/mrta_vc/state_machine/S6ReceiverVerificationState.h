/**
 *  S6ReceiverVerificationState.h
 *	
 *  Corresponds to S6 State in the State Machine Model Diagram
 *
 *  Version: 1.2.2
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_S6_RECEIVER_VERIFICATION_STATE_H_
#define TASK_BUILDER_S6_RECEIVER_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/PersonVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
    	class S6ReceiverVerificationState : public PersonVerificationState
		{

		public:
			S6ReceiverVerificationState(MachineController* controller);
			virtual ~S6ReceiverVerificationState();

			virtual bool process(std::string answer);
			virtual std::string toString();

    private:
			virtual bool next(std::string answer);

 		};
	}
}		
					
#endif /* TASK_BUILDER_S6_RECEIVER_VERIFICATION_STATE_H_ */
