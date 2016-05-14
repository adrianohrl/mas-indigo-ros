/**
 *  S7PriorityVerificationState.h
 *
 *  Version: 1.0.0.0
 *  Created on: 13/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef S7_PRIORITY_VERIFICATION_STATE_H_
#define S7_PRIORITY_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/AbstractState.h"
#include "unifei/expertinos/mrta_vc/tasks/TaskPriorities.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class S7PriorityVerificationState : public AbstractState
		{

		public:
			S7PriorityVerificationState(MachineController* controller);
 			~S7PriorityVerificationState();

     		virtual void process(std::string answer);

 		private:
          virtual void next(std::string answer);
		};
	}
}		
					
#endif /* S7_PRIORITY_VERIFICATION_STATE_H_ */
