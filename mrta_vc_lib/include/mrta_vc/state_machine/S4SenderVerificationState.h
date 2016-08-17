/**
 *  S4SenderVerificationState.h
 *	
 *  Corresponds to S4 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.0.0.0
 *  Created on: 13/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef S4_SENDER_VERIFICATION_STATE_H_
#define S4_SENDER_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/SenderVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class S4SenderVerificationState : public SenderVerificationState
		{

		public:
			S4SenderVerificationState(MachineController* controller);
			virtual ~S4SenderVerificationState();

			virtual bool process(std::string answer);
			virtual std::string toString();

    private:
			virtual bool next(std::string answer);

 		};
	}
}		
					
#endif /* S4_SENDER_VERIFICATION_STATE_H_ */
