/**
 *  This header file defines the HierarchyLevelEnum enumerateds and the
 *  HierarchyLevels helper class.
 *
 *  Version: 1.4.0
 *  Created on: 21/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _AGENTS_HIERARCHY_LEVELS_H_
#define _AGENTS_HIERARCHY_LEVELS_H_

#include <string>
#include <vector>

namespace mas
{
namespace agents
{
namespace levels
{
enum HierarchyLevelEnum
{
  VISITOR,
  INTERN,
  EMPLOYEE,
  SUPERVISOR,
  MIDDLE_MANAGER,
  SENIOR_MANAGER,
  CEO
};
}

typedef levels::HierarchyLevelEnum HierarchyLevelEnum;

class HierarchyLevels
{

public:
  static HierarchyLevelEnum toEnumerated(int code);
  static int toCode(HierarchyLevelEnum enumerated);
  static std::string str(HierarchyLevelEnum enumerated);
  static const char* c_str(HierarchyLevelEnum enumerated);
  static HierarchyLevelEnum getDefault();
  static std::vector<HierarchyLevelEnum> getAll();
  static int compare(HierarchyLevelEnum level1, HierarchyLevelEnum level2);
  static bool isHigher(HierarchyLevelEnum level1, HierarchyLevelEnum level2);
};
}
}

#endif /* _AGENTS_HIERARCHY_LEVELS_H_ */
