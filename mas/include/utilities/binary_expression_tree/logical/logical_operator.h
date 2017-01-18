/**
 *  This header file defines the Logical Operator abstract class.
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

#ifndef _UTILITIES_BET_LOGICAL_OPERATOR_H_
#define _UTILITIES_BET_LOGICAL_OPERATOR_H_

#include "utilities/binary_expression_tree/operator.h"

namespace utilities
{

namespace binary_expression_tree
{

namespace logical
{

template <typename E> class LogicalOperator : public Operator<bool, E>
{
};
}
}
}

#endif /* _UTILITIES_BET_LOGICAL_OPERATOR_H_ */
