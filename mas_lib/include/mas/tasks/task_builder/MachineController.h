/**
 *  MachineController.h
 *
 *  Version: 1.2.4
 *  Created on: 10/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_BUILDER_MACHINE_CONTROLLER_H_
#define TASK_BUILDER_MACHINE_CONTROLLER_H_

#include <string>
#include <ros/ros.h>
#include "mas/tasks/Task.h"
#include "mas/agents/User.h"
#include "mas/tasks/task_builder/AbstractState.h"
#include "mas/tasks/task_builder/S0InitialState.h"
#include "mas/tasks/task_builder/S1TaskVerificationState.h"
#include "mas/tasks/task_builder/S2TaskVerificationState.h"
#include "mas/tasks/task_builder/S3TaskVerificationState.h"
#include "mas/tasks/task_builder/S4SenderVerificationState.h"
#include "mas/tasks/task_builder/S5SenderVerificationState.h"
#include "mas/tasks/task_builder/S6ReceiverVerificationState.h"
#include "mas/tasks/task_builder/S7PriorityVerificationState.h"
#include "mas/tasks/task_builder/S8DeadlineVerificationState.h"
#include "mas/tasks/task_builder/S9FinalState.h"

namespace mas
{
	namespace tasks
	{
		namespace task_builder
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
		 		tasks::Task getTask();
				agents::User getUser();
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
				void setTask(tasks::Task task);
				void setTaskSender(agents::Person sender);
				void setTaskReceiver(agents::Person receiver);
				void setTaskPriority(tasks::TaskPriorityEnum priority);
				void setTaskDeadline(ros::Time deadline);
				void setTaskDeadline(ros::Duration duration);
				bool process(std::string answer);
		 		void reset();
				std::string toString();

			protected:
				void setUser(agents::User user);

	 		private:
				ros::NodeHandle nh_;
				tasks::Task task_;
				agents::User user_;
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
}		
					
#endif /* TASK_BUILDER_MACHINE_CONTROLLER_H_ */
