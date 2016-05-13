/**
 *  S6ReceiverVerificationState.h
 *	
 *  Corresponds to S6 State in the State Machine Model Diagram
 *
 *  Version: 1.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef S6_RECEIVER_VERIFICATION_STATE_H_
#define S6_RECEIVER_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/PersonVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
    	class S6ReceiverVerificationState : public PersonVerificationState
		{

		public:
			S6ReceiverVerificationState(MachineController* controller);
			~S6ReceiverVerificationState();

 			virtual void process(std::string answer);

    private:
      virtual void next();

 		};
	}
}		
					
#endif /* S6_RECEIVER_VERIFICATION_STATE_H_ */
