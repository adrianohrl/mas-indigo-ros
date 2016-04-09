/**
 *  TaskPriorities.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/TaskPriorities.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum unifei::expertinos::mrta_vc::tasks::TaskPriorities::toEnumerated(int code)
{
	unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum enumerated;
	switch (code)
	{
		case 1:
			enumerated = unifei::expertinos::mrta_vc::tasks::priorities::LOW;
			break;
		case 2:
			enumerated = unifei::expertinos::mrta_vc::tasks::priorities::NORMAL;
			break;
		case 3:
			enumerated = unifei::expertinos::mrta_vc::tasks::priorities::IMPORTANT;
			break;
		case 4:
			enumerated = unifei::expertinos::mrta_vc::tasks::priorities::CRITICAL;
			break;
		default:
			enumerated = getDefault();
	}
	return enumerated;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::TaskPriorities::toCode(unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum enumerated)
{
	int code;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::tasks::priorities::LOW:
			code = 1;
			break;
		case unifei::expertinos::mrta_vc::tasks::priorities::NORMAL:
			code = 2;
			break;
		case unifei::expertinos::mrta_vc::tasks::priorities::IMPORTANT:
			code = 3;
			break;
		case unifei::expertinos::mrta_vc::tasks::priorities::CRITICAL:
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
std::string unifei::expertinos::mrta_vc::tasks::TaskPriorities::toString(unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum enumerated)
{
	std::string enumerated_name;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::tasks::priorities::LOW:
			enumerated_name = "LOW";
			break;
		case unifei::expertinos::mrta_vc::tasks::priorities::NORMAL:
			enumerated_name = "NORMAL";
			break;
		case unifei::expertinos::mrta_vc::tasks::priorities::IMPORTANT:
			enumerated_name = "IMPORTANT";
			break;
		case unifei::expertinos::mrta_vc::tasks::priorities::CRITICAL:
			enumerated_name = "CRITICAL";
			break;
		default:
			enumerated_name = toString(getDefault());
	}
	return enumerated_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum unifei::expertinos::mrta_vc::tasks::TaskPriorities::getDefault()
{
	return unifei::expertinos::mrta_vc::tasks::priorities::LOW;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum> unifei::expertinos::mrta_vc::tasks::TaskPriorities::getAll()
{
	std::vector<unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum> enumerateds;
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::priorities::LOW);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::priorities::NORMAL);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::priorities::IMPORTANT);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::priorities::CRITICAL);
	return enumerateds;
}
