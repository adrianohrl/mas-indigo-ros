/**
 *  MachineController.h
 *
 *  Version: 1.0.0.0
 *  Created on: 10/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef MACHINE_CONTROLLER_H_
#define MACHINE_CONTROLLER_H_

#include <string>
#include <ros/ros.h> 
#include "unifei/expertinos/mrta_vc/tasks/Task.h"
//#include "mrta_vc/state_machine/AbstractState.h"
//#include "mrta_vc/state_machine/S8ReceiverVerificationState.h"

namespace mrta_vc
{
	namespace state_machine
	{
		class MachineController 
		{

		public:
			MachineController(ros::NodeHandle nh);	
 			~MachineController();

 			ros::NodeHandle getNodeHandle();
            unifei::expertinos::mrta_vc::tasks::Task getTask();
            //S8ReceiverVerificationState getS8();

            //void setNext(AbstractState state);

 		private:
 			ros::NodeHandle nh_;
            unifei::expertinos::mrta_vc::tasks::Task task_;
            //AbstractState current_;
            //S8ReceiverVerificationState s8_;


		};
	}
}		
					
#endif /* MACHINE_CONTROLLER_H_ */
