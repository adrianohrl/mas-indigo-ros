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
 			virtual void next();
            virtual bool isValid();

 		protected:
            TaskVerificationState(MachineController controller, std::string question = "What?");
            unifei::expertinos::mrta_vc::tasks::Task getTask();

 		private:
 			ros::ServiceClient get_task_cli_;
 			unifei::expertinos::mrta_vc::tasks::Task task_;
            bool valid_;
 		};
	}
}		
					
#endif /* TASK_VERIFICATION_STATE_H_ */
