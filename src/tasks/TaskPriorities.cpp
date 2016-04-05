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
unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum unifei::expertinos::mrta_vc::tasks::TaskPriorities::toEnumerated(int code)
{
	unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum enumerated;
	switch (code)
	{
		case 1:
			enumerated = LOW;
			break;
		case 2:
			enumerated = NORMAL;
			break;
		case 3:
			enumerated = IMPORTANT;
			break;
		case 4:
			enumerated = CRITICAL;
			break;
		default:
			enumerated = getDefault();
	}
	return enumerated;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::TaskPriorities::toCode(unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum enumerated)
{
	int code;
	switch (enumerated)
	{
		case LOW:
			code = 1;
			break;
		case NORMAL:
			code = 2;
			break;
		case IMPORTANT:
			code = 3;
			break;
		case CRITICAL:
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
std::string unifei::expertinos::mrta_vc::tasks::TaskPriorities::toString(unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum enumerated)
{
	std::string enumerated_name;
	switch (enumerated)
	{
		case LOW:
			enumerated_name = "LOW";
			break;
		case NORMAL:
			enumerated_name = "NORMAL";
			break;
		case IMPORTANT:
			enumerated_name = "IMPORTANT";
			break;
		case CRITICAL:
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
unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum unifei::expertinos::mrta_vc::tasks::TaskPriorities::getDefault()
{
	return LOW;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum> unifei::expertinos::mrta_vc::tasks::TaskPriorities::getAll()
{
	std::vector<TaskPriorityEnum> enumerateds;
	enumerateds.push_back(LOW);
	enumerateds.push_back(NORMAL);
	enumerateds.push_back(IMPORTANT);
	enumerateds.push_back(CRITICAL);
	return enumerateds;
}
