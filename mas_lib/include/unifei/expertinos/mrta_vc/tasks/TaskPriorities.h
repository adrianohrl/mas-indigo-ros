/**
 *  TaskPriorities.h
 *
 *  Version: 1.2.2
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASKS_TASK_PRIORITIES_H_
#define TASKS_TASK_PRIORITIES_H_

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
          static TaskPriorityEnum toEnumerated(std::string name);
          static bool isValid(std::string name);
					static int toCode(TaskPriorityEnum enumerated);
					static std::string toString(TaskPriorityEnum enumerated);
					static TaskPriorityEnum getDefault();
					static std::vector<TaskPriorityEnum> getAll();
          static int compare(TaskPriorityEnum priority1, TaskPriorityEnum priority2);

				};
			}
		}
	}
}



#endif /* TASKS_TASK_PRIORITIES_H_ */
