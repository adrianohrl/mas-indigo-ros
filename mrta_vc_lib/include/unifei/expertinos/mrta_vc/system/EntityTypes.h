/**
 *  EntityTypes.h
 *
 *  Version: 1.0.0.0
 *  Created on: 08/06/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef ENTITY_TYPES_H_
#define ENTITY_TYPES_H_

#include <string>
#include <vector>

namespace unifei 
{
	namespace expertinos
	{
		namespace mrta_vc
		{
			namespace system
			{
				namespace types
				{
					typedef enum 
					{
						AGENT,
						ALLOCATION,
						PLACE,
						RESOURCE,
						SKILL,
						TASK
					} EntityTypeEnum;
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
	}
}



#endif /* ENTITY_TYPES_H_ */
