/**
 *  AllocationStates.h
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ALLOCATION_STATES_H_
#define ALLOCATION_STATES_H_

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
				namespace states
				{
					typedef enum 
					{
						NOT_ALLOCATED,
						ALLOCATED,
						DISPATCHED,
						EXECUTING,
						CANCELLED,
						SUCCEEDED,
						ABORTED
					} AllocationStateEnum;
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
	}
}



#endif /* ALLOCATION_STATES_H_ */
