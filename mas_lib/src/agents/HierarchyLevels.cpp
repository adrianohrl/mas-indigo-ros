/**
 *  HierarchyLevels.cpp
 *
 *  Version: 1.2.2
 *  Created on: 21/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/agents/HierarchyLevels.h"

/**
 *
 */
unifei::expertinos::mrta_vc::agents::levels::HierarchyLevelEnum unifei::expertinos::mrta_vc::agents::HierarchyLevels::toEnumerated(int code)
{
	unifei::expertinos::mrta_vc::agents::levels::HierarchyLevelEnum enumerated;
	switch (code)
	{
		case 0:
			enumerated = unifei::expertinos::mrta_vc::agents::levels::VISITOR;
			break;
		case 1:
			enumerated = unifei::expertinos::mrta_vc::agents::levels::INTERN;
			break;
		case 2:
			enumerated = unifei::expertinos::mrta_vc::agents::levels::EMPLOYEE;
			break;
		case 3:
			enumerated = unifei::expertinos::mrta_vc::agents::levels::SUPERVISOR;
			break;
		case 4:
			enumerated = unifei::expertinos::mrta_vc::agents::levels::MIDDLE_MANAGER;
			break;
		case 5:
			enumerated = unifei::expertinos::mrta_vc::agents::levels::SENIOR_MANAGER;
			break;
		case 6:
			enumerated = unifei::expertinos::mrta_vc::agents::levels::CEO;
			break;
		default:
			enumerated = getDefault();
	}
	return enumerated;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::agents::HierarchyLevels::toCode(unifei::expertinos::mrta_vc::agents::levels::HierarchyLevelEnum enumerated)
{
	int code;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::agents::levels::VISITOR:
			code = 0;
			break;
		case unifei::expertinos::mrta_vc::agents::levels::INTERN:
			code = 1;
			break;
		case unifei::expertinos::mrta_vc::agents::levels::EMPLOYEE:
			code = 2;
			break;
		case unifei::expertinos::mrta_vc::agents::levels::SUPERVISOR:
			code = 3;
			break;
		case unifei::expertinos::mrta_vc::agents::levels::MIDDLE_MANAGER:
			code = 4;
			break;
		case unifei::expertinos::mrta_vc::agents::levels::SENIOR_MANAGER:
			code = 5;
			break;
		case unifei::expertinos::mrta_vc::agents::levels::CEO:
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
std::string unifei::expertinos::mrta_vc::agents::HierarchyLevels::toString(unifei::expertinos::mrta_vc::agents::levels::HierarchyLevelEnum enumerated)
{
	std::string enumerated_name;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::agents::levels::VISITOR:
			enumerated_name = "VISITOR";
			break;
		case unifei::expertinos::mrta_vc::agents::levels::INTERN:
			enumerated_name = "INTERN";
			break;
		case unifei::expertinos::mrta_vc::agents::levels::EMPLOYEE:
			enumerated_name = "EMPLOYEE";
			break;
		case unifei::expertinos::mrta_vc::agents::levels::SUPERVISOR:
			enumerated_name = "SUPERVISOR";
			break;
		case unifei::expertinos::mrta_vc::agents::levels::MIDDLE_MANAGER:
			enumerated_name = "MIDDLE_MANAGER";
			break;
		case unifei::expertinos::mrta_vc::agents::levels::SENIOR_MANAGER:
			enumerated_name = "MIDDLE_MANAGER";
			break;
		case unifei::expertinos::mrta_vc::agents::levels::CEO:
			enumerated_name = "CEO";
			break;
		default:
			enumerated_name = toString(getDefault());
	}
	return enumerated_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::agents::levels::HierarchyLevelEnum unifei::expertinos::mrta_vc::agents::HierarchyLevels::getDefault()
{
	return unifei::expertinos::mrta_vc::agents::levels::EMPLOYEE;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::agents::levels::HierarchyLevelEnum> unifei::expertinos::mrta_vc::agents::HierarchyLevels::getAll()
{
	std::vector<levels::HierarchyLevelEnum> enumerateds;
	enumerateds.push_back(unifei::expertinos::mrta_vc::agents::levels::VISITOR);
	enumerateds.push_back(unifei::expertinos::mrta_vc::agents::levels::INTERN);
	enumerateds.push_back(unifei::expertinos::mrta_vc::agents::levels::EMPLOYEE);
	enumerateds.push_back(unifei::expertinos::mrta_vc::agents::levels::SUPERVISOR);
	enumerateds.push_back(unifei::expertinos::mrta_vc::agents::levels::MIDDLE_MANAGER);
	enumerateds.push_back(unifei::expertinos::mrta_vc::agents::levels::SENIOR_MANAGER);
	enumerateds.push_back(unifei::expertinos::mrta_vc::agents::levels::CEO);
	return enumerateds;
}

/**
 *
 */
int unifei::expertinos::mrta_vc::agents::HierarchyLevels::compare(unifei::expertinos::mrta_vc::agents::levels::HierarchyLevelEnum level1, unifei::expertinos::mrta_vc::agents::levels::HierarchyLevelEnum level2)
{
	return toCode(level1) - toCode(level2);
}

/**
 *
 */
bool unifei::expertinos::mrta_vc::agents::HierarchyLevels::isHigher(unifei::expertinos::mrta_vc::agents::levels::HierarchyLevelEnum level1, unifei::expertinos::mrta_vc::agents::levels::HierarchyLevelEnum level2)
{
	return compare(level1, level2) > 0;
}
