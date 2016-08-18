/**
 *  AllocationStates.cpp
 *
 *  Version: 1.2.2
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/AllocationStates.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::states::AllocationStateEnum unifei::expertinos::mrta_vc::tasks::AllocationStates::toEnumerated(int code)
{
	unifei::expertinos::mrta_vc::tasks::states::AllocationStateEnum enumerated;
	switch (code)
	{
		case 0:
			enumerated = unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED;
			break;
		case 1:
			enumerated = unifei::expertinos::mrta_vc::tasks::states::ALLOCATED;
			break;
		case 2:
			enumerated = unifei::expertinos::mrta_vc::tasks::states::DISPATCHED;
			break;
		case 3:
			enumerated = unifei::expertinos::mrta_vc::tasks::states::EXECUTING;
			break;
		case 4:
			enumerated = unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED;
			break;
		case 5:
			enumerated = unifei::expertinos::mrta_vc::tasks::states::ABORTED;
			break;
		case 6:
			enumerated = unifei::expertinos::mrta_vc::tasks::states::CANCELLED;
			break;
		default:
			enumerated = getDefault();
	}
	return enumerated;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::AllocationStates::toCode(unifei::expertinos::mrta_vc::tasks::states::AllocationStateEnum enumerated)
{
	int code;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED:
			code = 0;
			break;
		case unifei::expertinos::mrta_vc::tasks::states::ALLOCATED:
			code = 1;
			break;
		case unifei::expertinos::mrta_vc::tasks::states::DISPATCHED:
			code = 2;
			break;
		case unifei::expertinos::mrta_vc::tasks::states::EXECUTING:
			code = 3;
			break;
		case unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED:
			code = 4;
			break;
		case unifei::expertinos::mrta_vc::tasks::states::ABORTED:
			code = 5;
			break;
		case unifei::expertinos::mrta_vc::tasks::states::CANCELLED:
			code = 6;
			break;
		default:			
			code = toCode(getDefault());
	}
	return code;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::tasks::AllocationStates::toString(unifei::expertinos::mrta_vc::tasks::states::AllocationStateEnum enumerated)
{
	std::string enumerated_name;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED:
			enumerated_name = "NOT_ALLOCATED";
			break;
		case unifei::expertinos::mrta_vc::tasks::states::ALLOCATED:
			enumerated_name = "ALLOCATED";
			break;
		case unifei::expertinos::mrta_vc::tasks::states::DISPATCHED:
			enumerated_name = "DISPATCHED";
			break;
		case unifei::expertinos::mrta_vc::tasks::states::EXECUTING:
			enumerated_name = "EXECUTING";
			break;
		case unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED:
			enumerated_name = "SUCCEEDED";
			break;
		case unifei::expertinos::mrta_vc::tasks::states::ABORTED:
			enumerated_name = "ABORTED";
			break;
		case unifei::expertinos::mrta_vc::tasks::states::CANCELLED:
			enumerated_name = "CANCELLED";
			break;
		default:
			enumerated_name = toString(getDefault());
	}
	return enumerated_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::states::AllocationStateEnum unifei::expertinos::mrta_vc::tasks::AllocationStates::getDefault()
{
	return unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::states::AllocationStateEnum> unifei::expertinos::mrta_vc::tasks::AllocationStates::getAll()
{
	std::vector<states::AllocationStateEnum> enumerateds;
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::states::NOT_ALLOCATED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::states::ALLOCATED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::states::DISPATCHED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::states::EXECUTING);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::states::SUCCEEDED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::states::ABORTED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::states::CANCELLED);
	return enumerateds;
}
