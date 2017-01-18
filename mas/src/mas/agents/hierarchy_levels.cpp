/**
 *  This source file implements the HierarchyLevels helper class.
 *
 *  Version: 1.4.0
 *  Created on: 21/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/hierarchy_levels.h"

namespace mas
{
namespace agents
{

/**
 * @brief HierarchyLevels::toEnumerated
 * @param code
 * @return
 */
levels::HierarchyLevelEnum HierarchyLevels::toEnumerated(int code)
{
  levels::HierarchyLevelEnum enumerated;
  switch (code)
  {
  case 0:
    enumerated = levels::VISITOR;
    break;
  case 1:
    enumerated = levels::INTERN;
    break;
  case 2:
    enumerated = levels::EMPLOYEE;
    break;
  case 3:
    enumerated = levels::SUPERVISOR;
    break;
  case 4:
    enumerated = levels::MIDDLE_MANAGER;
    break;
  case 5:
    enumerated = levels::SENIOR_MANAGER;
    break;
  case 6:
    enumerated = levels::CEO;
    break;
  default:
    enumerated = HierarchyLevels::getDefault();
  }
  return enumerated;
}

/**
 * @brief HierarchyLevels::toCode
 * @param enumerated
 * @return
 */
int HierarchyLevels::toCode(levels::HierarchyLevelEnum enumerated)
{
  int code;
  switch (enumerated)
  {
  case levels::VISITOR:
    code = 0;
    break;
  case levels::INTERN:
    code = 1;
    break;
  case levels::EMPLOYEE:
    code = 2;
    break;
  case levels::SUPERVISOR:
    code = 3;
    break;
  case levels::MIDDLE_MANAGER:
    code = 4;
    break;
  case levels::SENIOR_MANAGER:
    code = 5;
    break;
  case levels::CEO:
    code = 6;
    break;
  default:
    code = HierarchyLevels::toCode(HierarchyLevels::getDefault());
  }
  return code;
}

/**
 * @brief HierarchyLevels::c_str
 * @param enumerated
 * @return
 */
const char* HierarchyLevels::c_str(HierarchyLevelEnum enumerated)
{
  return HierarchyLevels::str(enumerated).c_str();
}

/**
 * @brief HierarchyLevels::str
 * @param enumerated
 * @return
 */
std::string HierarchyLevels::str(levels::HierarchyLevelEnum enumerated)
{
  std::string enumerated_name;
  switch (enumerated)
  {
  case levels::VISITOR:
    enumerated_name = "VISITOR";
    break;
  case levels::INTERN:
    enumerated_name = "INTERN";
    break;
  case levels::EMPLOYEE:
    enumerated_name = "EMPLOYEE";
    break;
  case levels::SUPERVISOR:
    enumerated_name = "SUPERVISOR";
    break;
  case levels::MIDDLE_MANAGER:
    enumerated_name = "MIDDLE_MANAGER";
    break;
  case levels::SENIOR_MANAGER:
    enumerated_name = "MIDDLE_MANAGER";
    break;
  case levels::CEO:
    enumerated_name = "CEO";
    break;
  default:
    enumerated_name = HierarchyLevels::str(HierarchyLevels::getDefault());
  }
  return enumerated_name;
}

/**
 * @brief HierarchyLevels::getDefault
 * @return
 */
levels::HierarchyLevelEnum HierarchyLevels::getDefault()
{
  return levels::EMPLOYEE;
}

/**
 * @brief HierarchyLevels::getAll
 * @return
 */
std::vector<levels::HierarchyLevelEnum> HierarchyLevels::getAll()
{
  std::vector<levels::HierarchyLevelEnum> enumerateds;
  enumerateds.push_back(levels::VISITOR);
  enumerateds.push_back(levels::INTERN);
  enumerateds.push_back(levels::EMPLOYEE);
  enumerateds.push_back(levels::SUPERVISOR);
  enumerateds.push_back(levels::MIDDLE_MANAGER);
  enumerateds.push_back(levels::SENIOR_MANAGER);
  enumerateds.push_back(levels::CEO);
  return enumerateds;
}

/**
 * @brief HierarchyLevels::compare
 * @param level1
 * @param level2
 * @return
 */
int HierarchyLevels::compare(levels::HierarchyLevelEnum level1,
                             levels::HierarchyLevelEnum level2)
{
  return HierarchyLevels::toCode(level1) - HierarchyLevels::toCode(level2);
}

/**
 * @brief HierarchyLevels::isHigher
 * @param level1
 * @param level2
 * @return
 */
bool HierarchyLevels::isHigher(levels::HierarchyLevelEnum level1,
                               levels::HierarchyLevelEnum level2)
{
  return HierarchyLevels::compare(level1, level2) > 0;
}
}
}
