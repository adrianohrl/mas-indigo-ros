/**
 *  SkillLevels.cpp
 *
 *  Version: 1.2.2
 *  Created on: 04/04/2016
 *  Modified on: 17/08/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/mrta_vc/system/EntityTypes.h"

/**
 *
 */
unifei::expertinos::mrta_vc::system::types::EntityTypeEnum unifei::expertinos::mrta_vc::system::EntityTypes::toEnumerated(int code)
{
	unifei::expertinos::mrta_vc::system::types::EntityTypeEnum enumerated;
  switch (code)
  {
    case 0:
			enumerated = unifei::expertinos::mrta_vc::system::types::AGENT;
      break;
    case 1:
			enumerated = unifei::expertinos::mrta_vc::system::types::ALLOCATION;
      break;
    case 2:
			enumerated = unifei::expertinos::mrta_vc::system::types::PLACE;
      break;
    case 3:
			enumerated = unifei::expertinos::mrta_vc::system::types::RESOURCE;
			break;
		case 4:
			enumerated = unifei::expertinos::mrta_vc::system::types::SKILL;
			break;
		case 5:
			enumerated = unifei::expertinos::mrta_vc::system::types::TASK;
			break;
    default:
      enumerated = getDefault();
  }
  return enumerated;
}

/**
 *
 */
unifei::expertinos::mrta_vc::system::types::EntityTypeEnum unifei::expertinos::mrta_vc::system::EntityTypes::toEnumerated(std::string name)
{
	unifei::expertinos::mrta_vc::system::types::EntityTypeEnum enumerated;
	if (name == "AGENT")
  {
		enumerated = unifei::expertinos::mrta_vc::system::types::AGENT;
  }
	else if (name == "ALLOCATION")
  {
		enumerated = unifei::expertinos::mrta_vc::system::types::ALLOCATION;
  }
	else if (name == "PLACE")
  {
		enumerated = unifei::expertinos::mrta_vc::system::types::PLACE;
  }
	else if (name == "RESOURCE")
	{
		enumerated = unifei::expertinos::mrta_vc::system::types::RESOURCE;
	}
	else if (name == "SKILL")
	{
		enumerated = unifei::expertinos::mrta_vc::system::types::SKILL;
	}
	else if (name == "TASK")
	{
		enumerated = unifei::expertinos::mrta_vc::system::types::TASK;
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
int unifei::expertinos::mrta_vc::system::EntityTypes::toCode(unifei::expertinos::mrta_vc::system::types::EntityTypeEnum enumerated)
{
	int code;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::system::types::AGENT:
			code = 0;
			break;
		case unifei::expertinos::mrta_vc::system::types::ALLOCATION:
			code = 1;
			break;
		case unifei::expertinos::mrta_vc::system::types::PLACE:
			code = 2;
			break;
		case unifei::expertinos::mrta_vc::system::types::RESOURCE:
			code = 3;
			break;
		case unifei::expertinos::mrta_vc::system::types::SKILL:
			code = 4;
			break;
		case unifei::expertinos::mrta_vc::system::types::TASK:
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
std::string unifei::expertinos::mrta_vc::system::EntityTypes::toString(unifei::expertinos::mrta_vc::system::types::EntityTypeEnum enumerated)
{
	std::string enumerated_name;
	switch (enumerated)
	{
		case unifei::expertinos::mrta_vc::system::types::AGENT:
			enumerated_name = "AGENT";
			break;
		case unifei::expertinos::mrta_vc::system::types::ALLOCATION:
			enumerated_name = "ALLOCATION";
			break;
		case unifei::expertinos::mrta_vc::system::types::PLACE:
			enumerated_name = "PLACE";
			break;
		case unifei::expertinos::mrta_vc::system::types::RESOURCE:
			enumerated_name = "RESOURCE";
			break;
		case unifei::expertinos::mrta_vc::system::types::SKILL:
			enumerated_name = "SKILL";
			break;
		case unifei::expertinos::mrta_vc::system::types::TASK:
			enumerated_name = "TASK";
			break;
		default:
			enumerated_name = toString(getDefault());
	}
	return enumerated_name;
}

/**
 *
 */
unifei::expertinos::mrta_vc::system::types::EntityTypeEnum unifei::expertinos::mrta_vc::system::EntityTypes::getDefault()
{
	return unifei::expertinos::mrta_vc::system::types::AGENT;
}

/**
 *
 */
std::vector<unifei::expertinos::mrta_vc::system::types::EntityTypeEnum> unifei::expertinos::mrta_vc::system::EntityTypes::getAll()
{
	std::vector<unifei::expertinos::mrta_vc::system::types::EntityTypeEnum> enumerateds;
	enumerateds.push_back(unifei::expertinos::mrta_vc::system::types::AGENT);
	enumerateds.push_back(unifei::expertinos::mrta_vc::system::types::ALLOCATION);
	enumerateds.push_back(unifei::expertinos::mrta_vc::system::types::PLACE);
	enumerateds.push_back(unifei::expertinos::mrta_vc::system::types::RESOURCE);
	enumerateds.push_back(unifei::expertinos::mrta_vc::system::types::SKILL);
	enumerateds.push_back(unifei::expertinos::mrta_vc::system::types::TASK);
	return enumerateds;
}
