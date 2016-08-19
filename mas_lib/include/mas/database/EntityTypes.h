/**
 *  EntityTypes.h
 *
 *  Version: 1.2.4
 *  Created on: 08/06/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef SYSTEM_ENTITY_TYPES_H_
#define SYSTEM_ENTITY_TYPES_H_

#include <string>
#include <vector>

namespace mas 
{
	namespace database
	{
		namespace types
		{
			enum EntityTypeEnum
			{
				AGENT,
				ALLOCATION,
				PLACE,
				RESOURCE,
				SKILL,
				TASK
			};
		}
		
		typedef types::EntityTypeEnum EntityTypeEnum;

		class EntityTypes
		{

		public:
			static EntityTypeEnum toEnumerated(int code);
			static EntityTypeEnum toEnumerated(std::string name);
			static int toCode(EntityTypeEnum enumerated);
			static std::string toString(EntityTypeEnum enumerated);
			static EntityTypeEnum getDefault();
			static std::vector<EntityTypeEnum> getAll();

		};
	}
}



#endif /* SYSTEM_ENTITY_TYPES_H_ */
