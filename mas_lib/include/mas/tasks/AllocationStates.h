/**
 *  AllocationStates.h
 *
 *  Version: 1.2.4
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASKS_ALLOCATION_STATES_H_
#define TASKS_ALLOCATION_STATES_H_

#include <string>
#include <vector>

namespace mas 
{
	namespace tasks
	{
		namespace states
		{
			enum AllocationStateEnum
			{
				NOT_ALLOCATED,
				ALLOCATED,
				DISPATCHED,
				EXECUTING,
				CANCELLED,
				SUCCEEDED,
				ABORTED
			};
		}
		
		typedef states::AllocationStateEnum AllocationStateEnum;

		class AllocationStates
		{

		public:
			static AllocationStateEnum toEnumerated(int code);
			static int toCode(AllocationStateEnum enumerated);
			static std::string toString(AllocationStateEnum enumerated);
			static AllocationStateEnum getDefault();
			static std::vector<AllocationStateEnum> getAll();

		};
	}
}



#endif /* TASKS_ALLOCATION_STATES_H_ */
