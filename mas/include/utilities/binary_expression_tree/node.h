/**
 *  This header file defines the Node interface.
 *
 *  Version: 1.4.0
 *  Created on: 05/09/2016
 *  Modified on: 14/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_BET_NODE_H_
#define _UTILITIES_BET_NODE_H_

#include <sstream>

namespace utilities
{

namespace binary_expression_tree
{

template <typename T, typename E> class Node
{

public:
  virtual T process() = 0;
  virtual bool hasLeft() const = 0;
  virtual bool hasRight() const = 0;
  virtual bool isLeaf() const = 0;
  virtual std::string str() const = 0; // in-order traversal
  virtual Node<T, E>* clone() = 0;
};
}
}

#endif /* _UTILITIES_BET_NODE_H_ */
