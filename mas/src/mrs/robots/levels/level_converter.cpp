/**
 *  This source file implements the Level Converter class, which is based on
 *  the Enum Converter Utility class.
 *
 *  Version: 1.4.0
 *  Created on: 16/09/2016
 *  Modified on: 14/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrs/robots/levels/level_converter.h"

namespace mrs
{

namespace robots
{

namespace levels
{

/**
 * @brief LevelConverter::LevelConverter
 * @param code
 */
LevelConverter::LevelConverter(int code) : EnumConverter(getEnumerated(code)) {}

/**
 * @brief LevelConverter::LevelConverter
 * @param name
 */
LevelConverter::LevelConverter(std::string name)
    : EnumConverter(getEnumerated(name))
{
}

/**
 * @brief LevelConverter::LevelConverter
 * @param level
 */
LevelConverter::LevelConverter(Level level) : EnumConverter(level) {}

/**
 * @brief LevelConverter::~LevelConverter
 */
LevelConverter::~LevelConverter() {}

/**
 * @brief LevelConverter::getEnumerated
 * @param code
 * @return
 */
Level LevelConverter::getEnumerated(int code) const
{
  switch (code)
  {
  case 0:
    return levels::NONE;
  case 1:
    return levels::LOW;
  case 2:
    return levels::MODERATE;
  case 3:
    return levels::HIGH;
  case 4:
    return levels::RESOURCEFUL;
  }
  return LevelConverter::getDefault();
}

/**
 * @brief LevelConverter::getEnumerated
 * @param name
 * @return
 */
Level LevelConverter::getEnumerated(std::string name) const
{
  if (name == "NONE" || name == "None" || name == "none")
  {
    return levels::NONE;
  }
  else if (name == "LOW" || name == "Low" || name == "low")
  {
    return levels::LOW;
  }
  else if (name == "MODERATE" || name == "Moderate" || name == "moderate")
  {
    return levels::MODERATE;
  }
  else if (name == "HIGH" || name == "High" || name == "high")
  {
    return levels::HIGH;
  }
  else if (name == "RESOURCEFUL" || name == "Resourceful" ||
           name == "resourceful")
  {
    return levels::RESOURCEFUL;
  }
  return LevelConverter::getDefault();
}

/**
 * @brief LevelConverter::str
 * @param level
 * @return
 */
std::string LevelConverter::str(Level level) const
{
  switch (level)
  {
  case levels::NONE:
    return "NONE";
  case levels::LOW:
    return "LOW";
  case levels::MODERATE:
    return "MODERATE";
  case levels::HIGH:
    return "HIGH";
  case levels::RESOURCEFUL:
    return "RESOURCEFUL";
  }
  return "";
}

/**
 * @brief LevelConverter::c_str
 * @param level
 * @return
 */
const char* LevelConverter::c_str(Level level) const
{
  return str(level).c_str();
}

/**
 * @brief LevelConverter::toEnumerated
 * @param code
 * @return
 */
Level LevelConverter::toEnumerated(int code)
{
//  LevelConverter converter(code);
//  return converter.getEnumerated();
}

/**
 * @brief LevelConverter::toEnumerated
 * @param name
 * @return
 */
Level LevelConverter::toEnumerated(std::string name)
{
//  LevelConverter converter(name);
//  return converter.getEnumerated();
}

/**
 * @brief LevelConverter::toString
 * @param level
 * @return
 */
std::string LevelConverter::toString(Level level)
{
//  LevelConverter converter(level);
//  return converter.str();
}

/**
 * @brief LevelConverter::toCString
 * @param level
 * @return
 */
const char* LevelConverter::toCString(Level level)
{
//  LevelConverter converter(level);
//  return converter.c_str();
}

/**
 * @brief LevelConverter::getDefault
 * @return
 */
Level LevelConverter::getDefault() { return levels::NONE; }

/**
 * @brief LevelConverter::getAll
 * @return
 */
std::vector<Level> LevelConverter::getAll()
{
  std::vector<Level> levels;
  levels.push_back(levels::NONE);
  levels.push_back(levels::LOW);
  levels.push_back(levels::MODERATE);
  levels.push_back(levels::HIGH);
  levels.push_back(levels::RESOURCEFUL);
  return levels;
}

/**
 * @brief LevelConverter::getMinimum
 * @return
 */
Level LevelConverter::getMinimum() { return levels::LOW; }

/**
 * @brief LevelConverter::getMaximum
 * @return
 */
Level LevelConverter::getMaximum() { return levels::RESOURCEFUL; }
}
}
}
