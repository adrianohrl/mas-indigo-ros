/**
 *  This header file defines the Capability Operand class.
 *
 *  Version: 1.4.0
 *  Created on: 05/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_CAPABILITY_OPERAND_H_
#define _UTILITIES_CAPABILITY_OPERAND_H_

#include "mrs/robots/capability.h"
#include "utilities/binary_expression_tree/operand.h"

namespace utilities
{

namespace capability_expression_evaluator
{

class CapabilityOperand
    : public binary_expression_tree::Operand<bool, mrs::robots::Capability>
{

public:
  CapabilityOperand(mrs::robots::Capability* capability,
                    mrs::robots::Capability* desired_capability);
  CapabilityOperand(const CapabilityOperand& operand);
  virtual ~CapabilityOperand();
  virtual bool process();
  // virtual binary_expression_tree::Content<bool, mrs::robots::Capability>*
  // parse(std::string expression);
  virtual std::string str() const; // in-order traversal
  virtual Node<bool, mrs::robots::Capability>* clone();

private:
  mrs::robots::Capability* capability_;
};
}
}

#endif /* _UTILITIES_CAPABILITY_OPERAND_H_ */
