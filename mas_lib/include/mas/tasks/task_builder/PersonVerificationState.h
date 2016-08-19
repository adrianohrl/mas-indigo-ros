/**
 *  PersonVerificationState.h
 *
 *  Version: 1.2.4
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_PERSON_VERIFICATION_STATE_H_
#define TASK_BUILDER_PERSON_VERIFICATION_STATE_H_

#include <mas_srvs/GetPerson.h>
#include "mas/agents/Person.h"
#include "mas/tasks/task_builder/AbstractState.h"

namespace mas
{
	namespace tasks
	{
		namespace task_builder
		{
			class PersonVerificationState : public AbstractState
			{

			public:
				virtual ~PersonVerificationState();

				virtual bool process(std::string answer);
				virtual std::string toString();

	 		protected:
				PersonVerificationState(MachineController* controller, std::string question = "From whom?");

				agents::Person getPerson();

			private:
				ros::ServiceClient get_person_cli_;
	 			agents::Person person_;

				virtual bool next(std::string answer);
	 		};
		}
	}
}		
					
#endif /* TASK_BUILDER_PERSON_VERIFICATION_STATE_H_ */
