/**
 *  SkillLevels.cpp
 *
 *  Version: 1.2.4
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/SkillLevels.h"

namespace mas
{
	namespace tasks
	{

		/**
		 *
		 */
		SkillLevelEnum SkillLevels::toEnumerated(int code)
		{
		  SkillLevelEnum enumerated;
		  switch (code)
		  {
		    case 0:
			 enumerated = levels::NONE;
			 break;
		    case 1:
			 enumerated = levels::LOW;
			 break;
		    case 2:
			 enumerated = levels::MODERATE;
			 break;
		    case 3:
			 enumerated = levels::HIGH;
			 break;
		    case 4:
			 enumerated = levels::RESOURCEFUL;
			 break;
		    default:
			 enumerated = getDefault();
		  }
		  return enumerated;
		}

		/**
		 *
		 */
		SkillLevelEnum SkillLevels::toEnumerated(std::string name)
		{
		  SkillLevelEnum enumerated;
		  if (name == "NONE" || name == "None" || name == "none")
		  {
		    enumerated = levels::NONE;
		  }
		  else if (name == "LOW" || name == "Low" || name == "low")
		  {
		    enumerated = levels::LOW;
		  }
		  else if (name == "MODERATE" || name == "Moderate" || name == "moderate")
		  {
		    enumerated = levels::MODERATE;
		  }
		  else if (name == "HIGH" || name == "High" || name == "high")
		  {
		    enumerated = levels::HIGH;
		  }
		  else if (name == "RESOURCEFUL" || name == "Resourceful" || name == "resourceful")
		  {
		    enumerated = levels::RESOURCEFUL;
		  }
		  else
		  {
			 enumerated = getDefault();
		  }
		  return enumerated;
		}

		/**
		 *
		 */
		int SkillLevels::toCode(SkillLevelEnum enumerated)
		{
			int code;
			switch (enumerated)
			{
				case levels::NONE:
					code = 0;
					break;
				case levels::LOW:
					code = 1;
					break;
				case levels::MODERATE:
					code = 2;
					break;
				case levels::HIGH:
					code = 3;
					break;
				case levels::RESOURCEFUL:
					code = 4;
					break;
				default:			
					code = toCode(getDefault());
			}
			return code;
		}

		/**
		 *
		 */
		std::string SkillLevels::toString(SkillLevelEnum enumerated)
		{
			std::string enumerated_name;
			switch (enumerated)
			{
				case levels::NONE:
					enumerated_name = "NONE";
					break;
				case levels::LOW:
					enumerated_name = "LOW";
					break;
				case levels::MODERATE:
					enumerated_name = "MODERATE";
					break;
				case levels::HIGH:
					enumerated_name = "HIGH";
					break;
				case levels::RESOURCEFUL:
					enumerated_name = "RESOURCEFUL";
					break;
				default:
					enumerated_name = toString(getDefault());
			}
			return enumerated_name;
		}

		/**
		 *
		 */
		SkillLevelEnum SkillLevels::getDefault()
		{
			return levels::NONE;
		}

		/**
		 *
		 */
		std::vector<SkillLevelEnum> SkillLevels::getAll()
		{
			std::vector<SkillLevelEnum> enumerateds;
			enumerateds.push_back(levels::NONE);
			enumerateds.push_back(levels::LOW);
			enumerateds.push_back(levels::MODERATE);
			enumerateds.push_back(levels::HIGH);
			enumerateds.push_back(levels::RESOURCEFUL);
			return enumerateds;
		}

		/**
		 *
		 */
		int SkillLevels::compare(SkillLevelEnum level1, SkillLevelEnum level2)
		{
			return toCode(level1) - toCode(level2);
		}

		/**
		 *
		 */
		bool SkillLevels::isSufficient(SkillLevelEnum level, SkillLevelEnum desired_level)
		{
			return compare(level, desired_level) >= 0;
		}
		
	}
}
