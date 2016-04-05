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
unifei::expertinos::mrta_vc::tasks::SkillLevelEnum unifei::expertinos::mrta_vc::tasks::SkillLevels::toEnumerated(int code)
{
	unifei::expertinos::mrta_vc::tasks::SkillLevelEnum enumerated;
	switch (code)
	{
		case 0:
			enumerated = NONE;
			break;
		case 1:
			enumerated = LOW;
			break;
		case 2:
			enumerated = MODERATE;
			break;
		case 3:
			enumerated = HIGH;
			break;
		case 4:
			enumerated = RESOURCEFUL;
			break;
		default:
			enumerated = getDefault();
	}
	return enumerated;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::SkillLevels::toCode(unifei::expertinos::mrta_vc::tasks::SkillLevelEnum enumerated)
{
	int code;
	switch (enumerated)
	{
		case NONE:
			code = 0;
			break;
		case LOW:
			code = 1;
			break;
		case MODERATE:
			code = 2;
			break;
		case HIGH:
			code = 3;
			break;
		case RESOURCEFUL:
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
std::string unifei::expertinos::mrta_vc::tasks::SkillLevels::toString(unifei::expertinos::mrta_vc::tasks::SkillLevelEnum enumerated)
{
	std::string enumerated_name;
	switch (enumerated)
	{
		case NONE:
			enumerated_name = "NONE";
			break;
		case LOW:
			enumerated_name = "LOW";
			break;
		case MODERATE:
			enumerated_name = "MODERATE";
			break;
		case HIGH:
			enumerated_name = "HIGH";
			break;
		case RESOURCEFUL:
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
unifei::expertinos::mrta_vc::tasks::SkillLevelEnum unifei::expertinos::mrta_vc::tasks::SkillLevels::getDefault()
{
	return NONE;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::SkillLevelEnum> unifei::expertinos::mrta_vc::tasks::SkillLevels::getAll()
{
	std::vector<SkillLevelEnum> enumerateds;
	enumerateds.push_back(NONE);
	enumerateds.push_back(LOW);
	enumerateds.push_back(MODERATE);
	enumerateds.push_back(HIGH);
	enumerateds.push_back(RESOURCEFUL);
	return enumerateds;
}
