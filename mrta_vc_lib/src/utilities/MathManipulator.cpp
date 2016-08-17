/**
 *  MathManipulator.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 21/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "unifei/expertinos/utilities/MathManipulator.h"

/**
 *
 */
int unifei::expertinos::utilities::MathManipulator::getUnsignedRest(long dividend, int divisor)
{
	return (int) (dividend >= 0L
								? dividend % divisor
								: (divisor + (dividend % divisor)) % divisor);
}

/**
 *
 */
long unifei::expertinos::utilities::MathManipulator::getUnsignedDivision(long dividend, int divisor)
{
	return dividend >= 0L
			? dividend / divisor
			: (dividend / divisor) - (dividend % divisor == 0 ? 0 : 1);
}
