/**
 *  S9FinalState.h
 *
 *  Version: 1.0.0.0
 *  Created on: 15/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef S9_FINAL_STATE_H_
#define S9_FINAL_STATE_H_

#include "mrta_vc/state_machine/AbstractState.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class S9FinalState : public AbstractState
		{

		public:
			S9FinalState(MachineController* controller);
			virtual ~S9FinalState();

			virtual bool process(std::string answer);
			virtual std::string toString();

		private:
			virtual bool next(std::string answer);

		};
	}
}
                              
#endif /* S9_FINAL_STATE_H_ */
