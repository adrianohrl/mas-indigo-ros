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
#include "mrta_vc/GenerateNewId.h"
#include "mrta_vc/state_machine/AbstractState.h"
#include "unifei/expertinos/mrta_vc/tasks/Task.h"
#include "unifei/expertinos/mrta_vc/system/EntityTypes.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class TaskVerificationState : public AbstractState
		{

		public:
			virtual ~TaskVerificationState();

			virtual bool process(std::string answer);
			virtual std::string toString();

 		protected:
      TaskVerificationState(MachineController* controller, std::string question = "What?");

		private:
			ros::ServiceClient get_task_cli_;
			ros::ServiceClient generate_new_id_cli_;

			virtual bool next(std::string answer);

 		};
	}
}		
					
#endif /* TASK_VERIFICATION_STATE_H_ */
