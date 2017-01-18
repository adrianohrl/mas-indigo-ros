/**
 *  This header file defines the SkillLevelEnum enumerateds and the SkillLevels
 *helper class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASKS_SKILL_LEVELS_H_
#define _TASKS_SKILL_LEVELS_H_

#include <string>
#include <vector>

namespace mas
{
namespace tasks
{
namespace levels
{
enum SkillLevelEnum
{
  NONE,
  LOW,
  MODERATE,
  HIGH,
  RESOURCEFUL
};
}

typedef levels::SkillLevelEnum SkillLevelEnum;

class SkillLevels
{

public:
  static SkillLevelEnum toEnumerated(int code);
  static SkillLevelEnum toEnumerated(std::string name);
  static int toCode(SkillLevelEnum enumerated);
  static std::string str(SkillLevelEnum enumerated);
  static const char* c_str(SkillLevelEnum enumerated);
  static SkillLevelEnum getDefault();
  static std::vector<SkillLevelEnum> getAll();
  static int compare(SkillLevelEnum level1, SkillLevelEnum level2);
  static bool isSufficient(SkillLevelEnum level, SkillLevelEnum desired_level);
};
}
}

#endif /* _TASKS_SKILL_LEVELS_H_ */
