/**
 *  S9FinalState.h
 *
 *  Version: 1.2.2
 *  Created on: 15/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_S9_FINAL_STATE_H_
#define TASK_BUILDER_S9_FINAL_STATE_H_

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
                              
#endif /* TASK_BUILDER_S9_FINAL_STATE_H_ */
