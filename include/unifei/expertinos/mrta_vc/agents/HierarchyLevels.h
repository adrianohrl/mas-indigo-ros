/**
 *  HierarchyLevels.h
 *
 *  Version: 1.0.0.0
 *  Created on: 21/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef HIERARCHY_LEVELS_H_
#define HIERARCHY_LEVELS_H_

#include <string>
#include <vector>

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace agents
			{
				namespace levels
				{
					typedef enum 
					{
						VISITOR,
						INTERN,
						EMPLOYEE,
						SUPERVISOR,
						MIDDLE_MANAGER,
						SENIOR_MANAGER,
						CEO
					} HierarchyLevelEnum;
				}
				
				typedef levels::HierarchyLevelEnum HierarchyLevelEnum;

				class HierarchyLevels
				{

				public:
					static HierarchyLevelEnum toEnumerated(int code);
					static int toCode(HierarchyLevelEnum enumerated);
					static std::string toString(HierarchyLevelEnum enumerated);
					static HierarchyLevelEnum getDefault();
					static std::vector<HierarchyLevelEnum> getAll();
					static int compare(HierarchyLevelEnum level1, HierarchyLevelEnum level2);
					static bool isHigher(HierarchyLevelEnum level1, HierarchyLevelEnum level2);

				};
			}
		}
	}
}



#endif /* HIERARCHY_LEVELS_H_ */
