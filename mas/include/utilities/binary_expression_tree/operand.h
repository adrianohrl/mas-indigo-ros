/**
 *  This header file defines the Operand abstract class.
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

#ifndef _UTILITIES_BET_OPERAND_H_
#define _UTILITIES_BET_OPERAND_H_

#include "utilities/binary_expression_tree/node.h"

namespace utilities
{

namespace binary_expression_tree
{

template <typename T, typename E> class Operand : public Node<T, E>
{
public:
  Operand(E* operand);
  Operand(const Operand& operand);
  virtual ~Operand();
  // virtual T process();
  virtual bool hasLeft() const;
  virtual bool hasRight() const;
  virtual bool isLeaf() const;
  virtual std::string str() const;

private:
  E* operand_;
};

/**
 *
 */
template <typename T, typename E> Operand<T, E>::Operand(E* operand)
{
  operand_ = operand;
}

/**
 *
 */
template <typename T, typename E> Operand<T, E>::Operand(const Operand& operand)
{
}

/**
 *
 */
template <typename T, typename E> Operand<T, E>::~Operand()
{
  if (operand_)
  {
    delete operand_;
    operand_ = NULL;
  }
}

/**
 *
 */
template <typename T, typename E> bool Operand<T, E>::hasLeft() const
{
  return false;
}

/**
 *
 */
template <typename T, typename E> bool Operand<T, E>::hasRight() const
{
  return false;
}

/**
 *
 */
template <typename T, typename E> bool Operand<T, E>::isLeaf() const
{
  return true;
}

/**
 *
 */
template <typename T, typename E> std::string Operand<T, E>::str() const
{
  return operand_->str(); // trocar para operand_->str()
}
}
}

#endif /* _UTILITIES_BET_OPERAND_H_ */
