/**
 *  This source file implements the Capability Operand class.
 *
 *  Version: 1.4.0
 *  Created on: 08/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/capability_expression_evaluator/capability_operand.h"

namespace utilities
{

namespace capability_expression_evaluator
{

/**
 * @brief CapabilityOperand::CapabilityOperand
 * @param capability
 * @param desired_capability
 */
CapabilityOperand::CapabilityOperand(
    mrs::robots::Capability* capability,
    mrs::robots::Capability* desired_capability)
    : Operand(desired_capability)
{
  capability_ = capability;
}

/**
 * @brief CapabilityOperand::CapabilityOperand
 * @param evaluator
 */
CapabilityOperand::CapabilityOperand(const CapabilityOperand& evaluator)
    : Operand(evaluator)
{
  capability_ = NULL;
  if (evaluator.capability_)
  {
    capability_ = evaluator.capability_->clone();
  }
}

/**
 * @brief CapabilityOperand::~CapabilityOperand
 */
CapabilityOperand::~CapabilityOperand()
{
  if (capability_)
  {
    delete capability_;
    capability_ = NULL;
  }
}

/**
 * @brief CapabilityOperand::process
 * @return
 */
bool CapabilityOperand::process()
{
  return false; // return skill_ && desired_skill_ &&
                // skill_->isSufficient(*desired_skill_); //// verificar
                // funcionamento desse metodo
}

/**
 * @brief SkillOperand::parse
 * @param expression
 * @return
 */
/*binary_expression_tree::Content<bool, mas::tasks::Skill>*
CapabilityOperand::parse(std::string expression)
{
  // implementar ainda
}*/

/**
 * @brief CapabilityOperand::str
 * @return
 */
std::string CapabilityOperand::str() const
{
  return ""; // return desired_skill_ ? desired_skill_->toString() : "";
}

/**
 * @brief CapabilityOperand::clone
 * @return
 */
binary_expression_tree::Node<bool, mrs::robots::Capability>*
CapabilityOperand::clone()
{
  return new CapabilityOperand(*this);
}
}
}
