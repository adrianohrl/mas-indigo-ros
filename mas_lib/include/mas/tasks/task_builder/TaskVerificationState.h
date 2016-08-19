/**
 *  TaskVerificationState.h
 *
 *  Version: 1.2.4
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_TASK_VERIFICATION_STATE_H_
#define TASK_BUILDER_TASK_VERIFICATION_STATE_H_

#include <mas_srvs/GetTask.h>
#include <mas_srvs/GenerateNewId.h>
#include "mas/tasks/Task.h"
#include "mas/tasks/task_builder/AbstractState.h"
#include "mas/database/EntityTypes.h"

namespace mas
{
	namespace tasks
	{
		namespace task_builder
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
}		
					
#endif /* TASK_BUILDER_TASK_VERIFICATION_STATE_H_ */
