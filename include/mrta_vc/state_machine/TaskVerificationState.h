/**
 *  TaskVerificationState.h
 *
 *  Version: 1.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_VERIFICATION_STATE_H_
#define TASK_VERIFICATION_STATE_H_

#include "mrta_vc/GetTask.h"
#include "mrta_vc/state_machine/AbstractState.h"
#include "unifei/expertinos/mrta_vc/tasks/Task.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class TaskVerificationState : public AbstractState
		{

		public:
 			~TaskVerificationState();

      virtual void process(std::string answer);

 		protected:
      TaskVerificationState(MachineController* controller, std::string question = "What?");

 		private:
      ros::ServiceClient get_task_cli_;

      virtual void next();

 		};
	}
}		
					
#endif /* TASK_VERIFICATION_STATE_H_ */
