/**
 *  This source file implements the SkillLevels helper class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/skill_levels.h"

namespace mas
{
namespace tasks
{

/**
 * @brief SkillLevels::toEnumerated
 * @param code
 * @return
 */
SkillLevelEnum SkillLevels::toEnumerated(int code)
{
  SkillLevelEnum enumerated;
  switch (code)
  {
  case 0:
    enumerated = levels::NONE;
    break;
  case 1:
    enumerated = levels::LOW;
    break;
  case 2:
    enumerated = levels::MODERATE;
    break;
  case 3:
    enumerated = levels::HIGH;
    break;
  case 4:
    enumerated = levels::RESOURCEFUL;
    break;
  default:
    enumerated = SkillLevels::getDefault();
  }
  return enumerated;
}

/**
 * @brief SkillLevels::toEnumerated
 * @param name
 * @return
 */
SkillLevelEnum SkillLevels::toEnumerated(std::string name)
{
  SkillLevelEnum enumerated;
  if (name == "NONE" || name == "None" || name == "none")
  {
    enumerated = levels::NONE;
  }
  else if (name == "LOW" || name == "Low" || name == "low")
  {
    enumerated = levels::LOW;
  }
  else if (name == "MODERATE" || name == "Moderate" || name == "moderate")
  {
    enumerated = levels::MODERATE;
  }
  else if (name == "HIGH" || name == "High" || name == "high")
  {
    enumerated = levels::HIGH;
  }
  else if (name == "RESOURCEFUL" || name == "Resourceful" ||
           name == "resourceful")
  {
    enumerated = levels::RESOURCEFUL;
  }
  else
  {
    enumerated = SkillLevels::getDefault();
  }
  return enumerated;
}

/**
 * @brief SkillLevels::toCode
 * @param enumerated
 * @return
 */
int SkillLevels::toCode(SkillLevelEnum enumerated)
{
  int code;
  switch (enumerated)
  {
  case levels::NONE:
    code = 0;
    break;
  case levels::LOW:
    code = 1;
    break;
  case levels::MODERATE:
    code = 2;
    break;
  case levels::HIGH:
    code = 3;
    break;
  case levels::RESOURCEFUL:
    code = 4;
    break;
  default:
    code = SkillLevels::toCode(SkillLevels::getDefault());
  }
  return code;
}

/**
 * @brief SkillLevels::toString
 * @param enumerated
 * @return
 */
std::string SkillLevels::str(SkillLevelEnum enumerated)
{
  std::string enumerated_name;
  switch (enumerated)
  {
  case levels::NONE:
    enumerated_name = "NONE";
    break;
  case levels::LOW:
    enumerated_name = "LOW";
    break;
  case levels::MODERATE:
    enumerated_name = "MODERATE";
    break;
  case levels::HIGH:
    enumerated_name = "HIGH";
    break;
  case levels::RESOURCEFUL:
    enumerated_name = "RESOURCEFUL";
    break;
  default:
    enumerated_name = SkillLevels::str(SkillLevels::getDefault());
  }
  return enumerated_name;
}

/**
 * @brief SkillLevels::c_str
 * @param enumerated
 * @return
 */
const char* SkillLevels::c_str(SkillLevelEnum enumerated)
{
  return SkillLevels::str(enumerated).c_str();
}

/**
 * @brief SkillLevels::getDefault
 * @return
 */
SkillLevelEnum SkillLevels::getDefault() { return levels::NONE; }

/**
 * @brief SkillLevels::getAll
 * @return
 */
std::vector<SkillLevelEnum> SkillLevels::getAll()
{
  std::vector<SkillLevelEnum> enumerateds;
  enumerateds.push_back(levels::NONE);
  enumerateds.push_back(levels::LOW);
  enumerateds.push_back(levels::MODERATE);
  enumerateds.push_back(levels::HIGH);
  enumerateds.push_back(levels::RESOURCEFUL);
  return enumerateds;
}

/**
 * @brief SkillLevels::compare
 * @param level1
 * @param level2
 * @return
 */
int SkillLevels::compare(SkillLevelEnum level1, SkillLevelEnum level2)
{
  return SkillLevels::toCode(level1) - SkillLevels::toCode(level2);
}

/**
 * @brief SkillLevels::isSufficient
 * @param level
 * @param desired_level
 * @return
 */
bool SkillLevels::isSufficient(SkillLevelEnum level,
                               SkillLevelEnum desired_level)
{
  return SkillLevels::compare(level, desired_level) >= 0;
}
}
}
