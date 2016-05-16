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
#include "mrta_vc/state_machine/S0InitialState.h"
#include "mrta_vc/state_machine/S1TaskVerificationState.h"
#include "mrta_vc/state_machine/S2TaskVerificationState.h"
#include "mrta_vc/state_machine/S3TaskVerificationState.h"
#include "mrta_vc/state_machine/S4SenderVerificationState.h"
#include "mrta_vc/state_machine/S5SenderVerificationState.h"
#include "mrta_vc/state_machine/S6ReceiverVerificationState.h"
#include "mrta_vc/state_machine/S7PriorityVerificationState.h"
#include "mrta_vc/state_machine/S8DeadlineVerificationState.h"
#include "mrta_vc/state_machine/S9FinalState.h"

namespace mrta_vc
{
	namespace state_machine
  {
    class AbstractState;

		class MachineController 
		{

		public:
			MachineController(ros::NodeHandle nh);	
 			~MachineController();

			ros::NodeHandle getNodeHandle();
			std::string getQuestion();
			std::string getMessage();
			bool hasChangedState();
			bool isFinalState();
      unifei::expertinos::mrta_vc::tasks::Task getTask();
      S0InitialState getS0();
      S1TaskVerificationState getS1();
      S2TaskVerificationState getS2();
      S3TaskVerificationState getS3();
      S4SenderVerificationState getS4();
      S5SenderVerificationState getS5();
      S6ReceiverVerificationState getS6();
			S7PriorityVerificationState getS7();
			S8DeadlineVerificationState getS8();
      S9FinalState getS9();
      void setTask(unifei::expertinos::mrta_vc::tasks::Task task);
			void setNext(AbstractState state);
			void process(std::string answer);
      void reset();

 		private:
 			ros::NodeHandle nh_;
      unifei::expertinos::mrta_vc::tasks::Task task_;
      AbstractState* current_;
      S0InitialState s0_;
      S1TaskVerificationState s1_;
      S2TaskVerificationState s2_;
      S3TaskVerificationState s3_;
      S4SenderVerificationState s4_;
      S5SenderVerificationState s5_;
      S6ReceiverVerificationState s6_;
			S7PriorityVerificationState s7_;
			S8DeadlineVerificationState s8_;
      S9FinalState s9_;
			bool changed_state_;

		};
	}
}		
					
#endif /* MACHINE_CONTROLLER_H_ */
