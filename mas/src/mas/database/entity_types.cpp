/**
 *  This source file implements the SkillLevels class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 14/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/database/entity_types.h"

namespace mas
{
	namespace database
	{

    /**
     * @brief EntityTypes::toEnumerated
     * @param code
     * @return
     */
		EntityTypeEnum EntityTypes::toEnumerated(int code)
		{
			EntityTypeEnum enumerated;
		  switch (code)
		  {
		    case 0:
					enumerated = types::AGENT;
			 break;
		    case 1:
					enumerated = types::ALLOCATION;
			 break;
		    case 2:
					enumerated = types::PLACE;
			 break;
		    case 3:
					enumerated = types::RESOURCE;
					break;
				case 4:
					enumerated = types::SKILL;
					break;
				case 5:
					enumerated = types::TASK;
					break;
		    default:
			 enumerated = getDefault();
		  }
		  return enumerated;
		}

    /**
     * @brief EntityTypes::toEnumerated
     * @param name
     * @return
     */
		EntityTypeEnum EntityTypes::toEnumerated(std::string name)
		{
			EntityTypeEnum enumerated;
			if (name == "AGENT")
		  {
				enumerated = types::AGENT;
		  }
			else if (name == "ALLOCATION")
		  {
				enumerated = types::ALLOCATION;
		  }
			else if (name == "PLACE")
		  {
				enumerated = types::PLACE;
		  }
			else if (name == "RESOURCE")
			{
				enumerated = types::RESOURCE;
			}
			else if (name == "SKILL")
			{
				enumerated = types::SKILL;
			}
			else if (name == "TASK")
			{
				enumerated = types::TASK;
			}
		  else
		  {
			 enumerated = getDefault();
		  }
		  return enumerated;
		}

    /**
     * @brief EntityTypes::toCode
     * @param enumerated
     * @return
     */
		int EntityTypes::toCode(EntityTypeEnum enumerated)
		{
			int code;
			switch (enumerated)
			{
				case types::AGENT:
					code = 0;
					break;
				case types::ALLOCATION:
					code = 1;
					break;
				case types::PLACE:
					code = 2;
					break;
				case types::RESOURCE:
					code = 3;
					break;
				case types::SKILL:
					code = 4;
					break;
				case types::TASK:
					code = 5;
					break;
				default:			
					code = toCode(getDefault());
			}
			return code;
		}

    /**
     * @brief EntityTypes::toString
     * @param enumerated
     * @return
     */
		std::string EntityTypes::toString(EntityTypeEnum enumerated)
		{
			std::string enumerated_name;
			switch (enumerated)
			{
				case types::AGENT:
					enumerated_name = "AGENT";
					break;
				case types::ALLOCATION:
					enumerated_name = "ALLOCATION";
					break;
				case types::PLACE:
					enumerated_name = "PLACE";
					break;
				case types::RESOURCE:
					enumerated_name = "RESOURCE";
					break;
				case types::SKILL:
					enumerated_name = "SKILL";
					break;
				case types::TASK:
					enumerated_name = "TASK";
					break;
				default:
					enumerated_name = toString(getDefault());
			}
			return enumerated_name;
		}

    /**
     * @brief EntityTypes::getDefault
     * @return
     */
		EntityTypeEnum EntityTypes::getDefault()
		{
			return types::AGENT;
		}

    /**
     * @brief EntityTypes::getAll
     * @return
     */
		std::vector<EntityTypeEnum> EntityTypes::getAll()
		{
			std::vector<EntityTypeEnum> enumerateds;
			enumerateds.push_back(types::AGENT);
			enumerateds.push_back(types::ALLOCATION);
			enumerateds.push_back(types::PLACE);
			enumerateds.push_back(types::RESOURCE);
			enumerateds.push_back(types::SKILL);
			enumerateds.push_back(types::TASK);
			return enumerateds;
		}

	}
}
