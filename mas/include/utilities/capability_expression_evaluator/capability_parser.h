/**
 *  This header file defines the Capability Parser class.
 *
 *  Version: 1.4.0
 *  Created on: 19/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef UTILITIES_CAPABILITY_PARSER_H_
#define UTILITIES_CAPABILITY_PARSER_H_

#include <sstream>
#include "mrs/robots/capability.h"
#include "utilities/string_manipulator.h"

namespace utilities
{

namespace capability_expression_evaluator
{

class CapabilityParser
{
public:
  CapabilityParser();
  virtual ~CapabilityParser();
  mrs::robots::Capability* parse(std::string expression);
  std::string str(const mrs::robots::Capability& capability) const;
  const char* c_str(const mrs::robots::Capability& capability) const;
};
}
}

#endif /* UTILITIES_CAPABILITY_PARSER_H_ */
