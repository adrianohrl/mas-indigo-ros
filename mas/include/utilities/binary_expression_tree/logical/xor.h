/**
 *  This header file defines the Logical XOR Operator class.
 *
 * OBS.: The class implementation is stated in this file because it make its usage
 * easier (due to the template use).
 *
 *  Version: 1.4.0
 *  Created on: 05/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_BET_LOGICAL_XOR_OPERATOR_H_
#define _UTILITIES_BET_LOGICAL_XOR_OPERATOR_H_

#include "utilities/binary_expression_tree/logical/logical_operator.h"

namespace utilities
{

namespace binary_expression_tree
{

namespace logical
{

template<typename E>
class XOR : public LogicalOperator<E>
{

public:
  XOR(const XOR& xor_operator);
  virtual ~XOR();
  virtual bool process(const Operand<E>& left, const Operand<E>& right);
  virtual Content<bool, E>* parse(std::string expression);
  virtual std::string str();

protected:
  XOR();

};

/**
 *
 */
template<typename E>
XOR<E>::XOR()
{}

/**
 *
 */
template<typename E>
XOR<E>::XOR(const XOR& xor_operator)
{}

/**
 *
 */
template<typename E>
XOR<E>::~XOR()
{}

/**
 *
 */
template<typename E>
bool XOR<E>::process()//const Operand<E>& left, const Operand<E>& right)
{
  //return left && right && left.process() != right.process();
  return false;
}

/**
 *
 */
template<typename E>
Content<bool, E>* XOR<E>::parse(std::string expression)
{
  // implementar
  return NULL;
}

/**
 *
 */
template<typename E>
std::string XOR<E>::str()
{
  return "XOR";
}

}

}

}

#endif /* _UTILITIES_BET_LOGICAL_XOR_OPERATOR_H_ */
