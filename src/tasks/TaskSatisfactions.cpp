/**
 *  TaskSatisfactions.cpp
 *
 *  Version: 1.0.0.0
 *  Created on: 04/04/2016
 *  Modified on: *********
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/TaskSatisfactions.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum unifei::expertinos::mrta_vc::tasks::TaskSatisfactions::toEnumerated(int code)
{
	unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum enumerated;
	switch (code)
	{
		case -3:
			enumerated = VERY_DISSATISFIED;
			break;
		case -2:
			enumerated = DISSATISFIED;
			break;
		case -1:
			enumerated = SOMEWHAT_DISSATISFIED;
			break;
		case 0:
			enumerated = NEITHER_DISSATISFIED_NOR_SATISFIED;
			break;
		case 1:
			enumerated = SOMEWHAT_SATISFIED;
			break;
		case 2:
			enumerated = SATISFIED;
			break;
		case 3:
			enumerated = VERY_SATISFIED;
			break;
		default:
			enumerated = getDefault();
	}
	return enumerated;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::TaskSatisfactions::toCode(unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum enumerated)
{
	int code;
	switch (enumerated)
	{
		case VERY_DISSATISFIED:
		case HORRIBLE:
			code = -3;
			break;
		case DISSATISFIED:
		case VERY_BAD:
			code = -2;
			break;
		case SOMEWHAT_DISSATISFIED:
		case BAD:
			code = -1;
			break;
		case NEITHER_DISSATISFIED_NOR_SATISFIED:
		case FAIR_ENOUGH:
			code = 0;
			break;
		case SOMEWHAT_SATISFIED:
		case GOOD:
			code = 1;
			break;
		case SATISFIED:
		case VERY_GOOD:
			code = 2;
			break;
		case VERY_SATISFIED:
		case EXCELLENT:
			code = 3;
			break;
		default:			
			code = toCode(getDefault());
	}
	return code;
}

/**
 *
 */
std::string unifei::expertinos::mrta_vc::tasks::TaskSatisfactions::toString(unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum enumerated)
{
	std::string enumerated_name;
	switch (enumerated)
	{
		case VERY_DISSATISFIED:
			enumerated_name = "VERY_DISSATISFIED";
			break;
		case HORRIBLE:
			enumerated_name = "HORRIBLE";
			break;
		case DISSATISFIED:
			enumerated_name = "DISSATISFIED";
			break;
		case VERY_BAD:
			enumerated_name = "VERY_BAD";
			break;
		case SOMEWHAT_DISSATISFIED:
			enumerated_name = "SOMEWHAT_DISSATISFIED";
			break;
		case BAD:
			enumerated_name = "BAD";
			break;
		case NEITHER_DISSATISFIED_NOR_SATISFIED:
			enumerated_name = "NEITHER_DISSATISFIED_NOR_SATISFIED";
			break;
		case FAIR_ENOUGH:
			enumerated_name = "FAIR_ENOUGH";
			break;
		case SOMEWHAT_SATISFIED:
			enumerated_name = "SOMEWHAT_SATISFIED";
			break;
		case GOOD:
			enumerated_name = "GOOD";
			break;
		case SATISFIED:
			enumerated_name = "SATISFIED";
			break;
		case VERY_GOOD:
			enumerated_name = "VERY_GOOD";
			break;
		case VERY_SATISFIED:
			enumerated_name = "VERY_SATISFIED";
			break;
		case EXCELLENT:
			enumerated_name = "EXCELLENT";
			break;
		default:
			enumerated_name = toString(getDefault());
	}
	return enumerated_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum unifei::expertinos::mrta_vc::tasks::TaskSatisfactions::getDefault()
{
	return NEITHER_DISSATISFIED_NOR_SATISFIED;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::TaskSatisfactionEnum> unifei::expertinos::mrta_vc::tasks::TaskSatisfactions::getAll()
{
	std::vector<TaskSatisfactionEnum> enumerateds;
	enumerateds.push_back(VERY_DISSATISFIED);
	enumerateds.push_back(DISSATISFIED);
	enumerateds.push_back(SOMEWHAT_DISSATISFIED);
	enumerateds.push_back(NEITHER_DISSATISFIED_NOR_SATISFIED);
	enumerateds.push_back(SOMEWHAT_SATISFIED);
	enumerateds.push_back(SATISFIED);
	enumerateds.push_back(VERY_SATISFIED);
	return enumerateds;
}
