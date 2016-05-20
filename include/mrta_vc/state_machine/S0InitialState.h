/**
 *  S0InitialState.h
 *
 *  Corresponds to S0 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.0.0.0
 *  Created on: 13/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef S0_INITIAL_STATE_H_
#define S0_INITIAL_STATE_H_

#include "mrta_vc/state_machine/AbstractState.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class S0InitialState : public AbstractState
		{

		public:
			S0InitialState(MachineController* controller);
			virtual ~S0InitialState();

			virtual bool process(std::string answer);
			virtual std::string toString();

    private:
			virtual bool next(std::string answer);

 		};
	}
}		
					
#endif /* S0_INITIAL_STATE_H_ */
