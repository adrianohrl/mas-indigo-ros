/**
 *  AllocationSatisfactions.h
 *
 *  Version: 1.2.2
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef TASKS_ALLOCATION_SATISFACTIONS_H_
#define TASKS_ALLOCATION_SATISFACTIONS_H_

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
						NONE,
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
					} AllocationSatisfactionEnum;
				}
				
				typedef satisfactions::AllocationSatisfactionEnum AllocationSatisfactionEnum;

				class AllocationSatisfactions
				{
				public:
					static AllocationSatisfactionEnum toEnumerated(int code);
					static int toCode(AllocationSatisfactionEnum enumerated);
					static std::string toString(AllocationSatisfactionEnum enumerated);
					static AllocationSatisfactionEnum getDefault();
					static bool isValid(AllocationSatisfactionEnum enumerated);
					static std::vector<AllocationSatisfactionEnum> getAll();

				};
			}
		}
	}
}



#endif /* TASKS_ALLOCATION_SATISFACTIONS_H_ */
