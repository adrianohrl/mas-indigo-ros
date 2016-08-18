/**
 *  PersonVerificationState.h
 *
 *  Version: 1.2.2
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_PERSON_VERIFICATION_STATE_H_
#define TASK_BUILDER_PERSON_VERIFICATION_STATE_H_

#include <mas_srvs/GetPerson.h>
#include "mrta_vc/state_machine/AbstractState.h"
#include "unifei/expertinos/mrta_vc/agents/Person.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class PersonVerificationState : public AbstractState
		{

		public:
			virtual ~PersonVerificationState();

			virtual bool process(std::string answer);
			virtual std::string toString();

 		protected:
			PersonVerificationState(MachineController* controller, std::string question = "From whom?");

			unifei::expertinos::mrta_vc::agents::Person getPerson();

		private:
			ros::ServiceClient get_person_cli_;
 			unifei::expertinos::mrta_vc::agents::Person person_;

			virtual bool next(std::string answer);
 		};
	}
}		
					
#endif /* TASK_BUILDER_PERSON_VERIFICATION_STATE_H_ */
