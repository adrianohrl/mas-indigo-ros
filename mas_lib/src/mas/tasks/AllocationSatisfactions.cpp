/**
 *  AllocationSatisfactions.cpp
 *
 *  Version: 1.2.4
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/AllocationSatisfactions.h"

namespace mas
{
	namespace tasks
	{

		/**
		 *
		 */
		satisfactions::AllocationSatisfactionEnum AllocationSatisfactions::toEnumerated(int code)
		{
			satisfactions::AllocationSatisfactionEnum enumerated;
			switch (code)
			{
				case -100:
					enumerated = satisfactions::NONE;
					break;
				case -3:
					enumerated = satisfactions::VERY_DISSATISFIED;
					break;
				case -2:
					enumerated = satisfactions::DISSATISFIED;
					break;
				case -1:
					enumerated = satisfactions::SOMEWHAT_DISSATISFIED;
					break;
				case 0:
					enumerated = satisfactions::NEITHER_DISSATISFIED_NOR_SATISFIED;
					break;
				case 1:
					enumerated = satisfactions::SOMEWHAT_SATISFIED;
					break;
				case 2:
					enumerated = satisfactions::SATISFIED;
					break;
				case 3:
					enumerated = satisfactions::VERY_SATISFIED;
					break;
				default:
					enumerated = getDefault();
			}
			return enumerated;
		}

		/**
		 *
		 */
		int AllocationSatisfactions::toCode(satisfactions::AllocationSatisfactionEnum enumerated)
		{
			int code;
			switch (enumerated)
			{
				case satisfactions::NONE:
					code = -100;
					break;
				case satisfactions::VERY_DISSATISFIED:
				case satisfactions::HORRIBLE:
					code = -3;
					break;
				case satisfactions::DISSATISFIED:
				case satisfactions::VERY_BAD:
					code = -2;
					break;
				case satisfactions::SOMEWHAT_DISSATISFIED:
				case satisfactions::BAD:
					code = -1;
					break;
				case satisfactions::NEITHER_DISSATISFIED_NOR_SATISFIED:
				case satisfactions::FAIR_ENOUGH:
					code = 0;
					break;
				case satisfactions::SOMEWHAT_SATISFIED:
				case satisfactions::GOOD:
					code = 1;
					break;
				case satisfactions::SATISFIED:
				case satisfactions::VERY_GOOD:
					code = 2;
					break;
				case satisfactions::VERY_SATISFIED:
				case satisfactions::EXCELLENT:
					code = 3;
					break;
				default:			
					code = toCode(getDefault());
			}
			return code;
		}

		/**
		 *
		 */
		std::string AllocationSatisfactions::toString(satisfactions::AllocationSatisfactionEnum enumerated)
		{
			std::string enumerated_name;
			switch (enumerated)
			{
				case satisfactions::NONE:
					enumerated_name = "NONE";
					break;
				case satisfactions::VERY_DISSATISFIED:
					enumerated_name = "VERY_DISSATISFIED";
					break;
				case satisfactions::HORRIBLE:
					enumerated_name = "HORRIBLE";
					break;
				case satisfactions::DISSATISFIED:
					enumerated_name = "DISSATISFIED";
					break;
				case satisfactions::VERY_BAD:
					enumerated_name = "VERY_BAD";
					break;
				case satisfactions::SOMEWHAT_DISSATISFIED:
					enumerated_name = "SOMEWHAT_DISSATISFIED";
					break;
				case satisfactions::BAD:
					enumerated_name = "BAD";
					break;
				case satisfactions::NEITHER_DISSATISFIED_NOR_SATISFIED:
					enumerated_name = "NEITHER_DISSATISFIED_NOR_SATISFIED";
					break;
				case satisfactions::FAIR_ENOUGH:
					enumerated_name = "FAIR_ENOUGH";
					break;
				case satisfactions::SOMEWHAT_SATISFIED:
					enumerated_name = "SOMEWHAT_SATISFIED";
					break;
				case satisfactions::GOOD:
					enumerated_name = "GOOD";
					break;
				case satisfactions::SATISFIED:
					enumerated_name = "SATISFIED";
					break;
				case satisfactions::VERY_GOOD:
					enumerated_name = "VERY_GOOD";
					break;
				case satisfactions::VERY_SATISFIED:
					enumerated_name = "VERY_SATISFIED";
					break;
				case satisfactions::EXCELLENT:
					enumerated_name = "EXCELLENT";
					break;
				default:
					enumerated_name = toString(getDefault());
			}
			return enumerated_name;
		}

		/**
		 *
		 */
		satisfactions::AllocationSatisfactionEnum AllocationSatisfactions::getDefault()
		{
			return satisfactions::NONE;
		}

		/**
		 *
		 */
		bool AllocationSatisfactions::isValid(satisfactions::AllocationSatisfactionEnum enumerated)
		{
			return enumerated != satisfactions::NONE;
		}

		/**
		 *
		 */
		std::vector<satisfactions::AllocationSatisfactionEnum> AllocationSatisfactions::getAll()
		{
			std::vector<satisfactions::AllocationSatisfactionEnum> enumerateds;
			enumerateds.push_back(satisfactions::VERY_DISSATISFIED);
			enumerateds.push_back(satisfactions::DISSATISFIED);
			enumerateds.push_back(satisfactions::SOMEWHAT_DISSATISFIED);
			enumerateds.push_back(satisfactions::NEITHER_DISSATISFIED_NOR_SATISFIED);
			enumerateds.push_back(satisfactions::SOMEWHAT_SATISFIED);
			enumerateds.push_back(satisfactions::SATISFIED);
			enumerateds.push_back(satisfactions::VERY_SATISFIED);
			return enumerateds;
		}
		
	}
}
