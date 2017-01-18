/**
 *  This header file defines the MathManipulator helper class.
 *
 *  Version: 1.4.0
 *  Created on: 21/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_MATH_MANIPULATOR_H_
#define _UTILITIES_MATH_MANIPULATOR_H_

#include <math.h>

namespace utilities
{
class MathManipulator
{

public:
  static int getUnsignedRest(long dividend, int divisor);
  static long getUnsignedDivision(long dividend, int divisor);
};
}

#endif /* _UTILITIES_MATH_MANIPULATOR_H_ */
