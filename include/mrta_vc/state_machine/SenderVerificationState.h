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
			virtual ~SenderVerificationState();

			virtual bool process(std::string answer);
			virtual std::string toString();

 		protected:
			SenderVerificationState(MachineController* controller);

		private:
			virtual bool next(std::string answer);

 		};
	}
}		
					
#endif /* SENDER_VERIFICATION_STATE_H_ */
