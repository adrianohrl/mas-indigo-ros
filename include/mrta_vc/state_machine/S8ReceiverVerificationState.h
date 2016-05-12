/**
 *  S8ReceiverVerificationState.h
 *	
 *	Corresponds to S8 State in the State Machine Model Diagram
 *
 *  Version: 1.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef S8_RECEIVER_VERIFICATION_STATE_H_
#define S8_RECEIVER_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/PersonVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class S8ReceiverVerificationState : public PersonVerificationState
		{

		public:
            S8ReceiverVerificationState(MachineController controller);
 			~S8ReceiverVerificationState();

 			virtual void process(std::string answer);
            virtual void next();	
 		};
	}
}		
					
#endif /* S8_RECEIVER_VERIFICATION_STATE_H_ */
