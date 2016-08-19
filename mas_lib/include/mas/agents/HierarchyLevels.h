/**
 *  HierarchyLevels.h
 *
 *  Version: 1.2.4
 *  Created on: 21/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef AGENTS_HIERARCHY_LEVELS_H_
#define AGENTS_HIERARCHY_LEVELS_H_

#include <string>
#include <vector>

namespace mas 
{
	namespace agents
	{
		namespace levels
		{
			enum HierarchyLevelEnum
			{
				VISITOR,
				INTERN,
				EMPLOYEE,
				SUPERVISOR,
				MIDDLE_MANAGER,
				SENIOR_MANAGER,
				CEO
			};
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



#endif /* AGENTS_HIERARCHY_LEVELS_H_ */
