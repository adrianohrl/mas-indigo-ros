/**
 *  This header file defines the Binary Expression Tree class.
 *
 * OBS.: The class implementation is stated in this file because it make its
 *usage
 * easier (due to the template use).
 *
 *  Version: 1.4.0
 *  Created on: 08/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_BET_H_
#define _UTILITIES_BET_H_

#include "utilities/binary_expression_tree/node.h"

namespace utilities
{

namespace binary_expression_tree
{

template <typename T, typename E> class BinaryExpressionTree
{

public:
  BinaryExpressionTree(std::string expression);
  BinaryExpressionTree(const BinaryExpressionTree<T, E>& tree);
  virtual ~BinaryExpressionTree();
  virtual T process();
  virtual std::string str() const; // in-order traversal

protected:
  Node<T, E>* getRoot() const;
  bool hasRoot() const;

private:
  Node<T, E>* root_;
};

/**
 *
 */
template <typename T, typename E>
BinaryExpressionTree<T, E>::BinaryExpressionTree(std::string expression)
{
  root_ = NULL;
}

/**
 *
 */
template <typename T, typename E>
BinaryExpressionTree<T, E>::BinaryExpressionTree(
    const BinaryExpressionTree<T, E>& tree)
{
  root_ = NULL;
  if (tree.root_)
  {
    root_ = tree.root_->clone();
  }
}

/**
 *
 */
template <typename T, typename E>
BinaryExpressionTree<T, E>::~BinaryExpressionTree()
{
  if (hasRoot())
  {
    delete root_;
  }
}

/**
 *
 */
template <typename T, typename E> T BinaryExpressionTree<T, E>::process()
{
  return hasRoot() ? root_->process() : T();
}

/**
 * In-order Traversal
 */
template <typename T, typename E> std::string BinaryExpressionTree<T, E>::str() const
{
  return hasRoot() ? root_->str() : "";
}

/**
 *
 */
template <typename T, typename E>
Node<T, E>* BinaryExpressionTree<T, E>::getRoot() const
{
  return root_;
}

/**
 *
 */
template <typename T, typename E> bool BinaryExpressionTree<T, E>::hasRoot() const
{
  return root_;
}
}
}

#endif /* _UTILITIES_BET_H_ */
