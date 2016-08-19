/**
 *  AllocationStates.cpp
 *
 *  Version: 1.2.4
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/AllocationStates.h"

namespace mas
{
	namespace tasks
	{

		/**
		 *
		 */
		states::AllocationStateEnum AllocationStates::toEnumerated(int code)
		{
			states::AllocationStateEnum enumerated;
			switch (code)
			{
				case 0:
					enumerated = states::NOT_ALLOCATED;
					break;
				case 1:
					enumerated = states::ALLOCATED;
					break;
				case 2:
					enumerated = states::DISPATCHED;
					break;
				case 3:
					enumerated = states::EXECUTING;
					break;
				case 4:
					enumerated = states::SUCCEEDED;
					break;
				case 5:
					enumerated = states::ABORTED;
					break;
				case 6:
					enumerated = states::CANCELLED;
					break;
				default:
					enumerated = getDefault();
			}
			return enumerated;
		}

		/**
		 *
		 */
		int AllocationStates::toCode(states::AllocationStateEnum enumerated)
		{
			int code;
			switch (enumerated)
			{
				case states::NOT_ALLOCATED:
					code = 0;
					break;
				case states::ALLOCATED:
					code = 1;
					break;
				case states::DISPATCHED:
					code = 2;
					break;
				case states::EXECUTING:
					code = 3;
					break;
				case states::SUCCEEDED:
					code = 4;
					break;
				case states::ABORTED:
					code = 5;
					break;
				case states::CANCELLED:
					code = 6;
					break;
				default:			
					code = toCode(getDefault());
			}
			return code;
		}

		/**
		 *
		 */
		std::string AllocationStates::toString(states::AllocationStateEnum enumerated)
		{
			std::string enumerated_name;
			switch (enumerated)
			{
				case states::NOT_ALLOCATED:
					enumerated_name = "NOT_ALLOCATED";
					break;
				case states::ALLOCATED:
					enumerated_name = "ALLOCATED";
					break;
				case states::DISPATCHED:
					enumerated_name = "DISPATCHED";
					break;
				case states::EXECUTING:
					enumerated_name = "EXECUTING";
					break;
				case states::SUCCEEDED:
					enumerated_name = "SUCCEEDED";
					break;
				case states::ABORTED:
					enumerated_name = "ABORTED";
					break;
				case states::CANCELLED:
					enumerated_name = "CANCELLED";
					break;
				default:
					enumerated_name = toString(getDefault());
			}
			return enumerated_name;
		}

		/**
		 *
		 */
		states::AllocationStateEnum AllocationStates::getDefault()
		{
			return states::NOT_ALLOCATED;
		}

		/**
		 *
		 */
		std::vector<states::AllocationStateEnum> AllocationStates::getAll()
		{
			std::vector<states::AllocationStateEnum> enumerateds;
			enumerateds.push_back(states::NOT_ALLOCATED);
			enumerateds.push_back(states::ALLOCATED);
			enumerateds.push_back(states::DISPATCHED);
			enumerateds.push_back(states::EXECUTING);
			enumerateds.push_back(states::SUCCEEDED);
			enumerateds.push_back(states::ABORTED);
			enumerateds.push_back(states::CANCELLED);
			return enumerateds;
		}
		
	}
}
