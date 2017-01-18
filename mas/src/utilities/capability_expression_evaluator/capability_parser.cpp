/**
 *  This source file implements the Skill Operand class.
 *
 *  Version: 1.4.0
 *  Created on: 08/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/capability_expression_evaluator/capability_parser.h"

namespace utilities
{

namespace capability_expression_evaluator
{

/**
 * @brief CapabilityParser::CapabilityParser
 */
CapabilityParser::CapabilityParser() {}

/**
 * @brief CapabilityParser::~CapabilityParser
 */
CapabilityParser::~CapabilityParser() {}

/**
 * @brief CapabilityParser::parse
 * @param expression
 * @return
 */
mrs::robots::Capability* CapabilityParser::parse(std::string expression)
{
  if (expression.find('<') != std::string::npos &&
          (expression.find('<') == 0 ||
           expression.rfind('<') + 1 >= expression.size() ||
           expression.rfind("<=") != std::string::npos &&
               expression.rfind("<=") + 2 >= expression.size()) ||
      expression.find("==") != std::string::npos &&
          (expression.find("==") == 0 ||
           expression.find("==") + 2 > expression.size()) ||
      expression.find('<') != std::string::npos &&
          (expression.find('>') == 0 ||
           expression.rfind('>') + 1 >= expression.size() ||
           expression.rfind(">=") != std::string::npos &&
               expression.rfind(">=") + 2 >= expression.size()))
  {
    return NULL;
  }
  std::string resource("");
  int lower_level(-1), upper_level(-1);
  std::size_t found_first(expression.find("=="));
  if (found_first != std::string::npos &&
      found_first == expression.rfind("==") &&
      expression.find('<') == std::string::npos &&
      expression.find('>') == std::string::npos) // resource == LEVEL
  {
    lower_level = atoi(expression.substr(found_first + 2).c_str());
    upper_level = lower_level;
    resource = expression.substr(0, found_first);
  }
  else
  {
    found_first = expression.find('>');
    if (found_first != std::string::npos &&
        found_first == expression.rfind('>') &&
        expression.find('<') == std::string::npos &&
        expression.find("==") == std::string::npos)
    {
      upper_level = mrs::robots::LevelConverter::getMaximum();
      resource = expression.substr(0, found_first);
      if (expression.find(">=") != std::string::npos) // resouce >= LEVEL
      {
        lower_level = atoi(expression.substr(found_first + 2).c_str());
      }
      else // resource > LEVEL
      {
        lower_level = atoi(expression.substr(found_first + 1).c_str()) + 1;
      }
    }
    else
    {
      found_first = expression.find('<');
      if (found_first != std::string::npos &&
          expression.find('>') == std::string::npos &&
          expression.find("==") == std::string::npos)
      {
        std::size_t found_last(expression.rfind('<'));
        if (found_first < found_last)
        {
          lower_level = atoi(expression.substr(0, found_first).c_str());
          if (expression.find("<=") ==
              std::string::npos) // LEVEL_1 < resource < LEVEL_2
          {
            lower_level++;
            upper_level = atoi(expression.substr(found_last + 1).c_str()) - 1;
            resource = expression.substr(found_first + 1,
                                         found_last - found_first - 1);
          }
          else
          {
            found_first = expression.find("<=");
            if (found_first == found_last) // LEVEL_1 < resource <= LEVEL_2
            {
              found_first = expression.find('<');
              lower_level++;
              upper_level = atoi(expression.substr(found_last + 2).c_str());
              resource = expression.substr(found_first + 1,
                                           found_last - found_first - 1);
            }
            else
            {
              found_last = expression.rfind("<=");
              if (found_first == found_last) // LEVEL_1 <= resource < LEVEL_2
              {
                found_last = expression.rfind('<');
                upper_level =
                    atoi(expression.substr(found_last + 1).c_str()) - 1;
                resource = expression.substr(found_first + 2,
                                             found_last - found_first - 2);
              }
              else // LEVEL_1 <= resource <= LEVEL_2
              {
                upper_level = atoi(expression.substr(found_last + 2).c_str());
                resource = expression.substr(found_first + 2,
                                             found_last - found_first - 2);
              }
            }
          }
        }
        else
        {
          lower_level = mrs::robots::LevelConverter::getMinimum();
          resource = expression.substr(0, found_first);
          if (expression.find("<=") != std::string::npos) // resource <= LEVEL
          {
            upper_level = atoi(expression.substr(found_first + 2).c_str());
          }
          else // resource < LEVEL
          {
            upper_level = atoi(expression.substr(found_first + 2).c_str()) - 1;
          }
        }
      }
      else
      {
        return NULL;
      }
    }
  }
  while (!resource.empty() && resource[0] == ' ')
  {
    resource = resource.substr(1);
  }
  while (!resource.empty() && resource[resource.size() - 1] == ' ')
  {
    resource = resource.substr(0, resource.size() - 1);
  }
  std::cout << "\nlower: " << lower_level << ", upper: " << upper_level
            << ", resource: " + resource + "\n";
  if (resource.empty() || lower_level > upper_level ||
      lower_level < mrs::robots::LevelConverter::getMinimum() ||
      lower_level > mrs::robots::LevelConverter::getMaximum() ||
      upper_level < mrs::robots::LevelConverter::getMinimum() ||
      upper_level > mrs::robots::LevelConverter::getMaximum())
  {
    return NULL;
  }
  return new mrs::robots::Capability(
      0, mrs::robots::Resource(resource),
      mrs::robots::LevelConverter::toEnumerated(lower_level),
      mrs::robots::LevelConverter::toEnumerated(upper_level));
}

/**
 * @brief CapabilityParser::str
 * @param capability
 * @return
 */
std::string CapabilityParser::str(const mrs::robots::Capability& capability) const
{
  if (capability.getLowerLevel() == mrs::robots::levels::NONE)
  {
    return "";
  }
  std::stringstream ss;
  /*if (capability.getLowerLevel() == mrs::robots::levels::LOW)
  {

  }
  else if (capability.getUpperLevel() == mrs::robots::levels::RESOURCEFUL)
  {

  }*/
  ss << capability.getLowerLevel();
  ss << " <= " + capability.getResource()->getName() + " <= ";
  ss << capability.getUpperLevel();
  return ss.str();
}

/**
 * @brief CapabilityParser::c_str
 * @param capability
 * @return
 */
const char* CapabilityParser::c_str(const mrs::robots::Capability& capability) const
{
  return str(capability).c_str();
}
}
}
