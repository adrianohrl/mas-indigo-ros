/**
 *  TaskStates.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/TaskStates.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::TaskStateEnum unifei::expertinos::mrta_vc::tasks::TaskStates::toEnumerated(int code)
{
	unifei::expertinos::mrta_vc::tasks::TaskStateEnum enumerated;
	switch (code)
	{
		case 0:
			enumerated = NOT_ALLOCATED;
			break;
		case 1:
			enumerated = WAITING_ACCEPTATION;
			break;
		case 2:
			enumerated = EXECUTING;
			break;
		case 3:
			enumerated = SUCCEEDED;
			break;
		case 4:
			enumerated = ABORTED;
			break;
		case 5:
			enumerated = FAILED;
			break;
		default:
			enumerated = getDefault();
	}
	return enumerated;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::TaskStates::toCode(unifei::expertinos::mrta_vc::tasks::TaskStateEnum enumerated)
{
	int code;
	switch (enumerated)
	{
		case NOT_ALLOCATED:
			code = 0;
			break;
		case WAITING_ACCEPTATION:
			code = 1;
			break;
		case EXECUTING:
			code = 2;
			break;
		case SUCCEEDED:
			code = 3;
			break;
		case ABORTED:
			code = 4;
			break;
		case FAILED:
			code = 5;
			break;
		default:			
			code = toCode(getDefault());
	}
	return code;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::tasks::TaskStates::toString(unifei::expertinos::mrta_vc::tasks::TaskStateEnum enumerated)
{
	std::string enumerated_name;
	switch (enumerated)
	{
		case NOT_ALLOCATED:
			enumerated_name = "NOT_ALLOCATED";
			break;
		case WAITING_ACCEPTATION:
			enumerated_name = "WAITING_ACCEPTATION";
			break;
		case EXECUTING:
			enumerated_name = "EXECUTING";
			break;
		case SUCCEEDED:
			enumerated_name = "SUCCEEDED";
			break;
		case ABORTED:
			enumerated_name = "ABORTED";
			break;
		case FAILED:
			enumerated_name = "FAILED";
			break;
		default:
			enumerated_name = toString(getDefault());
	}
	return enumerated_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::TaskStateEnum unifei::expertinos::mrta_vc::tasks::TaskStates::getDefault()
{
	return NOT_ALLOCATED;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::TaskStateEnum> unifei::expertinos::mrta_vc::tasks::TaskStates::getAll()
{
	std::vector<TaskStateEnum> enumerateds;
	enumerateds.push_back(NOT_ALLOCATED);
	enumerateds.push_back(WAITING_ACCEPTATION);
	enumerateds.push_back(EXECUTING);
	enumerateds.push_back(SUCCEEDED);
	enumerateds.push_back(ABORTED);
	enumerateds.push_back(FAILED);
	return enumerateds;
}
