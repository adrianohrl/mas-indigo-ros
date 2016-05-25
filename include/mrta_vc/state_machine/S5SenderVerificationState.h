/**
 *  S5SenderVerificationState.h
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

#ifndef S5_SENDER_VERIFICATION_STATE_H_
#define S5_SENDER_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/SenderVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
    class S5SenderVerificationState : public SenderVerificationState
		{

		public:
			S5SenderVerificationState(MachineController* controller);
			virtual ~S5SenderVerificationState();

			virtual bool process(std::string answer);
			virtual std::string toString();

    private:
			virtual bool next(std::string answer);

 		};
	}
}		
					
#endif /* S5_SENDER_VERIFICATION_STATE_H_ */
