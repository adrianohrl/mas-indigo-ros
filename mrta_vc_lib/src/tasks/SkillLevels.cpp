/**
 *  SkillLevels.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/SkillLevels.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum unifei::expertinos::mrta_vc::tasks::SkillLevels::toEnumerated(int code)
{
  unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum enumerated;
  switch (code)
  {
    case 0:
      enumerated = unifei::expertinos::mrta_vc::tasks::levels::NONE;
      break;
    case 1:
      enumerated = unifei::expertinos::mrta_vc::tasks::levels::LOW;
      break;
    case 2:
      enumerated = unifei::expertinos::mrta_vc::tasks::levels::MODERATE;
      break;
    case 3:
      enumerated = unifei::expertinos::mrta_vc::tasks::levels::HIGH;
      break;
    case 4:
      enumerated = unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL;
      break;
    default:
      enumerated = getDefault();
  }
  return enumerated;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum unifei::expertinos::mrta_vc::tasks::SkillLevels::toEnumerated(std::string name)
{
  unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum enumerated;
  if (name == "NONE" || name == "None" || name == "none")
  {
    enumerated = unifei::expertinos::mrta_vc::tasks::levels::NONE;
  }
  else if (name == "LOW" || name == "Low" || name == "low")
  {
    enumerated = unifei::expertinos::mrta_vc::tasks::levels::LOW;
  }
  else if (name == "MODERATE" || name == "Moderate" || name == "moderate")
  {
    enumerated = unifei::expertinos::mrta_vc::tasks::levels::MODERATE;
  }
  else if (name == "HIGH" || name == "High" || name == "high")
  {
    enumerated = unifei::expertinos::mrta_vc::tasks::levels::HIGH;
  }
  else if (name == "RESOURCEFUL" || name == "Resourceful" || name == "resourceful")
  {
    enumerated = unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL;
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
int unifei::expertinos::mrta_vc::tasks::SkillLevels::toCode(unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum enumerated)
{
	int code;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::tasks::levels::NONE:
			code = 0;
			break;
		case unifei::expertinos::mrta_vc::tasks::levels::LOW:
			code = 1;
			break;
		case unifei::expertinos::mrta_vc::tasks::levels::MODERATE:
			code = 2;
			break;
		case unifei::expertinos::mrta_vc::tasks::levels::HIGH:
			code = 3;
			break;
		case unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL:
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
std::string unifei::expertinos::mrta_vc::tasks::SkillLevels::toString(unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum enumerated)
{
	std::string enumerated_name;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::tasks::levels::NONE:
			enumerated_name = "NONE";
			break;
		case unifei::expertinos::mrta_vc::tasks::levels::LOW:
			enumerated_name = "LOW";
			break;
		case unifei::expertinos::mrta_vc::tasks::levels::MODERATE:
			enumerated_name = "MODERATE";
			break;
		case unifei::expertinos::mrta_vc::tasks::levels::HIGH:
			enumerated_name = "HIGH";
			break;
		case unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL:
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
unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum unifei::expertinos::mrta_vc::tasks::SkillLevels::getDefault()
{
	return unifei::expertinos::mrta_vc::tasks::levels::NONE;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum> unifei::expertinos::mrta_vc::tasks::SkillLevels::getAll()
{
	std::vector<unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum> enumerateds;
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::levels::NONE);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::levels::LOW);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::levels::MODERATE);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::levels::HIGH);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::levels::RESOURCEFUL);
	return enumerateds;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::SkillLevels::compare(unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum level1, unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum level2)
{
	return toCode(level1) - toCode(level2);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::SkillLevels::isSufficient(unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum level, unifei::expertinos::mrta_vc::tasks::levels::SkillLevelEnum desired_level)
{
	return compare(level, desired_level) >= 0;
}
