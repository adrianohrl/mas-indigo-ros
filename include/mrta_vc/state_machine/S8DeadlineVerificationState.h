/**
 *  S8DeadlineVerificationState.h
 *
 *  Version: 1.0.0.0
 *  Created on: 14/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef S8_DEADLINE_VERIFICATION_STATE_H_
#define S8_DEADLINE_VERIFICATION_STATE_H_

#include "mrta_vc/state_machine/AbstractState.h"
#include "unifei/expertinos/mrta_vc/tasks/TaskPriorities.h"

#define DEFAULT_DURATION 3600

namespace mrta_vc
{
	namespace state_machine
	{
		class S8DeadlineVerificationState : public AbstractState
		{

		public:
			S8DeadlineVerificationState(MachineController* controller);
 			~S8DeadlineVerificationState();

     		virtual void process(std::string answer);

 		private:
        virtual void next(std::string answer);
        bool isDeadline(std::string answer);
        bool isDuration(std::string answer);
        ros::Time getDeadline(std::string answer);
        ros::Duration getDuration(std::string answer);
		};
	}
}		
					
#endif /* S8_DEADLINE_VERIFICATION_STATE_H_ */
