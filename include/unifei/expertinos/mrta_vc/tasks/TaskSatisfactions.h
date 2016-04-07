/**
 *  TaskSatisfactions.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASK_SATISFACTIONS_H_
#define TASK_SATISFACTIONS_H_

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
				namespace satisfactions
				{
					typedef enum 
					{
						VERY_DISSATISFIED,
						HORRIBLE,
						DISSATISFIED,
						VERY_BAD,
						SOMEWHAT_DISSATISFIED,
						BAD,
						NEITHER_DISSATISFIED_NOR_SATISFIED,
						FAIR_ENOUGH,
						SOMEWHAT_SATISFIED,
						GOOD,
						SATISFIED,
						VERY_GOOD,
						VERY_SATISFIED,
						EXCELLENT
					} TaskSatisfactionEnum;
				}
				
				typedef satisfactions::TaskSatisfactionEnum TaskSatisfactionEnum;

				class TaskSatisfactions
				{

				public:
					static TaskSatisfactionEnum toEnumerated(int code);
					static int toCode(TaskSatisfactionEnum enumerated);
					static std::string toString(TaskSatisfactionEnum enumerated);
					static TaskSatisfactionEnum getDefault();
					static std::vector<TaskSatisfactionEnum> getAll();

				};
			}
		}
	}
}



#endif /* TASK_SATISFACTIONS_H_ */
