/**
 *  This header file defines the Logical NOT Operator class.
 *
 * OBS.: The class implementation is stated in this file because it make its
 *usage
 * easier (due to the template use).
 *
 *  Version: 1.4.0
 *  Created on: 05/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_BET_LOGICAL_NOT_OPERATOR_H_
#define _UTILITIES_BET_LOGICAL_NOT_OPERATOR_H_

#include "utilities/binary_expression_tree/logical/logical_operator.h"

namespace utilities
{

namespace binary_expression_tree
{

namespace logical
{

template <typename E> class NOT : public LogicalOperator<E>
{

public:
  NOT(const NOT& not_operator);
  virtual ~NOT();
  virtual bool process(const Operand<E>& left, const Operand<E>& right);
  virtual Content<bool, E>* parse(std::string expression);
  virtual std::string str() const;

protected:
  NOT();
};

/**
 *
 */
template <typename E> NOT<E>::NOT() {}

/**
 *
 */
template <typename E> NOT<E>::NOT(const NOT& not_operator) {}

/**
 *
 */
template <typename E> NOT<E>::~NOT() {}

/**
 *
 */
template <typename E>
bool NOT<E>::process() // const Operand<E>& left, const Operand<E>& right)
{
  // return left && !left.process() || right && !right.process();
  return false;
}

/**
 *
 */
template <typename E> Content<bool, E>* NOT<E>::parse(std::string expression)
{
  // implementar
  return NULL;
}

/**
 *
 */
template <typename E> std::string NOT<E>::str() const { return "NOT"; }
}
}
}

#endif /* _UTILITIES_BET_LOGICAL_NOT_OPERATOR_H_ */
