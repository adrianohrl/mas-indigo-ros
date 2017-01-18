/**
 *  This source file implements the MathManipulator class.
 *
 *  Version: 1.4.0
 *  Created on: 21/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/math_manipulator.h"

namespace utilities
{
	
/**
 * @brief MathManipulator::getUnsignedRest
 * @param dividend
 * @param divisor
 * @return
 */
int MathManipulator::getUnsignedRest(long dividend, int divisor)
{
  return (int) (dividend >= 0L ? dividend % divisor : (divisor + (dividend % divisor)) % divisor);
}

/**
 * @brief MathManipulator::getUnsignedDivision
 * @param dividend
 * @param divisor
 * @return
 */
long MathManipulator::getUnsignedDivision(long dividend, int divisor)
{
  return dividend >= 0L ? dividend / divisor : (dividend / divisor) - (dividend % divisor == 0 ? 0 : 1);
}
	
}
