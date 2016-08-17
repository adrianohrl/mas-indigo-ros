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
unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum unifei::expertinos::mrta_vc::tasks::TaskPriorities::toEnumerated(std::string name)
{
  unifei::expertinos::mrta_vc::tasks::priorities::TaskPriorityEnum enumerated;
  if (name == "LOW" || name == "Low" || name == "low")
  {
    enumerated = unifei::expertinos::mrta_vc::tasks::priorities::LOW;
  }
  else if (name == "NORMAL" || name == "Normal" || name == "normal")
  {
    enumerated = unifei::expertinos::mrta_vc::tasks::priorities::NORMAL;
  }
  else if (name == "IMPORTANT" || name == "Important" || name == "important")
  {
    enumerated = unifei::expertinos::mrta_vc::tasks::priorities::IMPORTANT;
  }
  else if (name == "CRITICAL" || name == "Critical" || name == "critical")
  {
    enumerated = unifei::expertinos::mrta_vc::tasks::priorities::CRITICAL;
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
bool unifei::expertinos::mrta_vc::tasks::TaskPriorities::isValid(std::string name)
{
  return name == "LOW"       || name == "Low"       || name == "low"      ||
         name == "NORMAL"    || name == "Normal"    || name == "normal"   ||
         name == "IMPORTANT" || name == "Important" || name == "important"||
         name == "CRITICAL"  || name == "Critical"  || name == "critical";
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

/**
 *
 */
int unifei::expertinos::mrta_vc::tasks::TaskPriorities::compare(unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum priority1, unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum priority2)
{
  int code1 = toCode(priority1);
  int code2 = toCode(priority2);
  return code1 - code2;
}
