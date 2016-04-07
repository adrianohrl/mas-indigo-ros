/**
 *  TaskPriorities.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_PRIORITIES_H_
#define TASK_PRIORITIES_H_

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
				namespace priorities
				{
					typedef enum 
					{
						LOW,
						NORMAL,
						IMPORTANT,
						CRITICAL

					} TaskPriorityEnum;
				}
				
				typedef priorities::TaskPriorityEnum TaskPriorityEnum;

				class TaskPriorities
				{

				public:
					static TaskPriorityEnum toEnumerated(int code);
					static int toCode(TaskPriorityEnum enumerated);
					static std::string toString(TaskPriorityEnum enumerated);
					static TaskPriorityEnum getDefault();
					static std::vector<TaskPriorityEnum> getAll();

				};
			}
		}
	}
}



#endif /* TASK_PRIORITIES_H_ */
