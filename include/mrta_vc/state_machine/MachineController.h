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
#include "unifei/expertinos/mrta_vc/agents/User.h"
#include "mrta_vc/state_machine/AbstractState.h"
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
		class MachineController 
		{

		public:
			MachineController(ros::NodeHandle nh);	
			virtual ~MachineController();

			ros::NodeHandle getNodeHandle();
			std::string getQuestion();
			std::string getMessage();
			bool isFinalState();
      unifei::expertinos::mrta_vc::tasks::Task getTask();
			unifei::expertinos::mrta_vc::agents::User getUser();
			void setNextToS0();
			void setNextToS1();
			void setNextToS2();
			void setNextToS3();
			void setNextToS4();
			void setNextToS5();
			void setNextToS6();
			void setNextToS7();
			void setNextToS8();
			void setNextToS9();
			void setTask(unifei::expertinos::mrta_vc::tasks::Task task);
			void setTaskSender(unifei::expertinos::mrta_vc::agents::Person sender);
			void setTaskReceiver(unifei::expertinos::mrta_vc::agents::Person receiver);
			void setTaskPriority(unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum priority);
			void setTaskDeadline(ros::Time deadline);
			void setTaskDeadline(ros::Duration duration);
			bool process(std::string answer);
      void reset();
			std::string toString();

		protected:
			void setUser(unifei::expertinos::mrta_vc::agents::User user);

 		private:
			ros::NodeHandle nh_;
      unifei::expertinos::mrta_vc::tasks::Task task_;
			unifei::expertinos::mrta_vc::agents::User user_;
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

		};
	}
}		
					
#endif /* MACHINE_CONTROLLER_H_ */
