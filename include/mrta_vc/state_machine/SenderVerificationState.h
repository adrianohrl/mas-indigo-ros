/**
 *  SenderVerificationState.h
 *
 *  Version: 1.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SENDER_VERIFICATION_STATE_H_
#define SENDER_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/PersonVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class SenderVerificationState : public PersonVerificationState
		{

		public:
 			~SenderVerificationState();

      virtual void process(std::string answer);

 		protected:
      SenderVerificationState(MachineController* controller);

    private:
      virtual void next();

 		};
	}
}		
					
#endif /* SENDER_VERIFICATION_STATE_H_ */
