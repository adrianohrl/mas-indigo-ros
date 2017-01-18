/**
 *  This header file defines the Capability Expression Evaluator class.
 *
 *  Version: 1.3.0
 *  Created on: 08/09/2016
 *  Modified on: 10/09/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef UTILITIES_CAPABILITY_EXPRESSION_EVALUATOR_H_
#define UTILITIES_CAPABILITY_EXPRESSION_EVALUATOR_H_

#include <vector>
#include "utilities/binary_expression_tree/binary_expression_tree.h"
#include "utilities/capability_expression_evaluator/capability_operand.h"

namespace utilities
{

namespace capability_expression_evaluator
{

class CapabilityExpressionEvaluator
    : public binary_expression_tree::BinaryExpressionTree<
          bool, mrs::robots::Capability>
{

public:
  CapabilityExpressionEvaluator(
      std::string expression,
      std::vector<mrs::robots::Capability> capabilities);
  CapabilityExpressionEvaluator(const CapabilityExpressionEvaluator& evaluator);
  virtual ~CapabilityExpressionEvaluator();
};
}
}

#endif /* UTILITIES_CAPABILITY_EXPRESSION_EVALUATOR_H_ */
