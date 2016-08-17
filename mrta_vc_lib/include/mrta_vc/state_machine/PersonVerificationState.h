/**
 *  PersonVerificationState.h
 *
 *  Version: 1.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef PERSON_VERIFICATION_STATE_H_
#define PERSON_VERIFICATION_STATE_H_

#include "mrta_vc/GetPerson.h"
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
					
#endif /* PERSON_VERIFICATION_STATE_H_ */
