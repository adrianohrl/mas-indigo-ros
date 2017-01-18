/**
 *  This header file defines the Operator abstract class.
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

#ifndef _UTILITIES_BET_OPERATOR_H_
#define _UTILITIES_BET_OPERATOR_H_

#include "utilities/binary_expression_tree/node.h"

namespace utilities
{

namespace binary_expression_tree
{

template <typename T, typename E> class Operator : public Node<T, E>
{
public:
  Operator(const Operator& operatorr);
  virtual ~Operator();

  virtual bool hasLeft() const;
  virtual bool hasRight() const;
  virtual bool isLeaf() const;

protected:
  Operator(Node<T, E>* left, Node<T, E>* right);

private:
  Node<T, E>* left_, *right_;
};

/**
 *
 */
template <typename T, typename E>
Operator<T, E>::Operator(Node<T, E>* left, Node<T, E>* right)
{
  left_ = left;
  right_ = right;
}

/**
 *
 */
template <typename T, typename E>
Operator<T, E>::Operator(const Operator& operatorr)
{
  left_ = NULL;
  if (operatorr.left_)
  {
    left_ = operatorr.left_->clone();
  }
  right_ = NULL;
  if (operatorr.right_)
  {
    right_ = operatorr.right_->clone();
  }
}

/**
 *
 */
template <typename T, typename E> Operator<T, E>::~Operator()
{
  if (left_)
  {
    delete left_;
    left_ = NULL;
  }
  if (right_)
  {
    delete right_;
    right_ = NULL;
  }
}

/**
 *
 */
template <typename T, typename E> bool Operator<T, E>::hasLeft() const
{
  return left_;
}

/**
 *
 */
template <typename T, typename E> bool Operator<T, E>::hasRight() const
{
  return right_;
}

/**
 *
 */
template <typename T, typename E> bool Operator<T, E>::isLeaf() const
{
  return false;
}
}
}

#endif /* _UTILITIES_BET_OPERATOR_H_ */
