/**
 *  AllocationSatisfactions.cpp
 *
 *  Version: 1.2.2
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/tasks/AllocationSatisfactions.h"

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::satisfactions::AllocationSatisfactionEnum unifei::expertinos::mrta_vc::tasks::AllocationSatisfactions::toEnumerated(int code)
{
	unifei::expertinos::mrta_vc::tasks::satisfactions::AllocationSatisfactionEnum enumerated;
	switch (code)
	{
		case -100:
			enumerated = unifei::expertinos::mrta_vc::tasks::satisfactions::NONE;
			break;
		case -3:
			enumerated = unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_DISSATISFIED;
			break;
		case -2:
			enumerated = unifei::expertinos::mrta_vc::tasks::satisfactions::DISSATISFIED;
			break;
		case -1:
			enumerated = unifei::expertinos::mrta_vc::tasks::satisfactions::SOMEWHAT_DISSATISFIED;
			break;
		case 0:
			enumerated = unifei::expertinos::mrta_vc::tasks::satisfactions::NEITHER_DISSATISFIED_NOR_SATISFIED;
			break;
		case 1:
			enumerated = unifei::expertinos::mrta_vc::tasks::satisfactions::SOMEWHAT_SATISFIED;
			break;
		case 2:
			enumerated = unifei::expertinos::mrta_vc::tasks::satisfactions::SATISFIED;
			break;
		case 3:
			enumerated = unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_SATISFIED;
			break;
		default:
			enumerated = getDefault();
	}
	return enumerated;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::AllocationSatisfactions::toCode(unifei::expertinos::mrta_vc::tasks::satisfactions::AllocationSatisfactionEnum enumerated)
{
	int code;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::tasks::satisfactions::NONE:
			code = -100;
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_DISSATISFIED:
		case unifei::expertinos::mrta_vc::tasks::satisfactions::HORRIBLE:
			code = -3;
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::DISSATISFIED:
		case unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_BAD:
			code = -2;
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::SOMEWHAT_DISSATISFIED:
		case unifei::expertinos::mrta_vc::tasks::satisfactions::BAD:
			code = -1;
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::NEITHER_DISSATISFIED_NOR_SATISFIED:
		case unifei::expertinos::mrta_vc::tasks::satisfactions::FAIR_ENOUGH:
			code = 0;
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::SOMEWHAT_SATISFIED:
		case unifei::expertinos::mrta_vc::tasks::satisfactions::GOOD:
			code = 1;
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::SATISFIED:
		case unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_GOOD:
			code = 2;
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_SATISFIED:
		case unifei::expertinos::mrta_vc::tasks::satisfactions::EXCELLENT:
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
std::string unifei::expertinos::mrta_vc::tasks::AllocationSatisfactions::toString(unifei::expertinos::mrta_vc::tasks::satisfactions::AllocationSatisfactionEnum enumerated)
{
	std::string enumerated_name;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::tasks::satisfactions::NONE:
			enumerated_name = "NONE";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_DISSATISFIED:
			enumerated_name = "VERY_DISSATISFIED";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::HORRIBLE:
			enumerated_name = "HORRIBLE";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::DISSATISFIED:
			enumerated_name = "DISSATISFIED";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_BAD:
			enumerated_name = "VERY_BAD";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::SOMEWHAT_DISSATISFIED:
			enumerated_name = "SOMEWHAT_DISSATISFIED";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::BAD:
			enumerated_name = "BAD";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::NEITHER_DISSATISFIED_NOR_SATISFIED:
			enumerated_name = "NEITHER_DISSATISFIED_NOR_SATISFIED";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::FAIR_ENOUGH:
			enumerated_name = "FAIR_ENOUGH";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::SOMEWHAT_SATISFIED:
			enumerated_name = "SOMEWHAT_SATISFIED";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::GOOD:
			enumerated_name = "GOOD";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::SATISFIED:
			enumerated_name = "SATISFIED";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_GOOD:
			enumerated_name = "VERY_GOOD";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_SATISFIED:
			enumerated_name = "VERY_SATISFIED";
			break;
		case unifei::expertinos::mrta_vc::tasks::satisfactions::EXCELLENT:
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
unifei::expertinos::mrta_vc::tasks::satisfactions::AllocationSatisfactionEnum unifei::expertinos::mrta_vc::tasks::AllocationSatisfactions::getDefault()
{
	return unifei::expertinos::mrta_vc::tasks::satisfactions::NONE;
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::tasks::AllocationSatisfactions::isValid(unifei::expertinos::mrta_vc::tasks::satisfactions::AllocationSatisfactionEnum enumerated)
{
	return enumerated != unifei::expertinos::mrta_vc::tasks::satisfactions::NONE;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::tasks::satisfactions::AllocationSatisfactionEnum> unifei::expertinos::mrta_vc::tasks::AllocationSatisfactions::getAll()
{
	std::vector<satisfactions::AllocationSatisfactionEnum> enumerateds;
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_DISSATISFIED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::satisfactions::DISSATISFIED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::satisfactions::SOMEWHAT_DISSATISFIED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::satisfactions::NEITHER_DISSATISFIED_NOR_SATISFIED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::satisfactions::SOMEWHAT_SATISFIED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::satisfactions::SATISFIED);
	enumerateds.push_back(unifei::expertinos::mrta_vc::tasks::satisfactions::VERY_SATISFIED);
	return enumerateds;
}
