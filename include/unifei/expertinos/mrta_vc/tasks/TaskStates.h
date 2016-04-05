/**
 *  TaskStates.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_STATES_H_
#define TASK_STATES_H_

#include <string>
#include <vector>

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace tasks
			{
				typedef enum 
				{
					NOT_ALLOCATED,
					WAITING_ACCEPTATION,
					EXECUTING,
					SUCCEEDED,
					ABORTED,
					FAILED
				} TaskStateEnum;

				class TaskStates
				{

				public:
					static TaskStateEnum toEnumerated(int code);
					static int toCode(TaskStateEnum enumerated);
					static std::string toString(TaskStateEnum enumerated);
					static TaskStateEnum getDefault();
					static std::vector<TaskStateEnum> getAll();

				};
			}
		}
	}
}



#endif /* TASK_STATES_H_ */
