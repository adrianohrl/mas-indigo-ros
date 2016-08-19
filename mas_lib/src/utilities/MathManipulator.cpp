/**
 *  MathManipulator.cpp
 *
 *  Version: 1.2.4
 *  Created on: 21/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/MathManipulator.h"

namespace utilities
{
	
	/**
	 *
	 */
	int MathManipulator::getUnsignedRest(long dividend, int divisor)
	{
		return (int) (dividend >= 0L ? dividend % divisor : (divisor + (dividend % divisor)) % divisor);
	}

	/**
	 *
	 */
	long MathManipulator::getUnsignedDivision(long dividend, int divisor)
	{
		return dividend >= 0L ? dividend / divisor : (dividend / divisor) - (dividend % divisor == 0 ? 0 : 1);
	}
	
}
