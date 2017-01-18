/**
 *  This source file implements the Skill Expression Evaluator class.
 *
 *  Version: 1.4.0
 *  Created on: 08/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/capability_expression_evaluator/capability_expression_evaluator.h"

namespace utilities
{

namespace capability_expression_evaluator
{

/**
 * @brief CapabilityExpressionEvaluator::CapabilityExpressionEvaluator
 * @param expression
 * @param capabilities
 */
CapabilityExpressionEvaluator::CapabilityExpressionEvaluator(
    std::string expression, std::vector<mrs::robots::Capability> capabilities)
    : BinaryExpressionTree(expression)
{
  // implementar
}

/**
 * @brief CapabilityExpressionEvaluator::CapabilityExpressionEvaluator
 * @param evaluator
 */
CapabilityExpressionEvaluator::CapabilityExpressionEvaluator(
    const CapabilityExpressionEvaluator& evaluator)
    : BinaryExpressionTree(evaluator)
{
  // implementar
}

/**
 * @brief CapabilityExpressionEvaluator::~CapabilityExpressionEvaluator
 */
CapabilityExpressionEvaluator::~CapabilityExpressionEvaluator() {}
}
}
