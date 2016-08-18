/**
 *  MathManipulator.h
 *
 *  Version: 1.2.2
 *  Created on: 21/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef UTILITIES_MATH_MANIPULATOR_H_
#define UTILITIES_MATH_MANIPULATOR_H_

#include <math.h>

namespace unifei
{
	namespace expertinos
	{
		namespace utilities
		{
			class MathManipulator
			{

			public:
				static int getUnsignedRest(long dividend, int divisor);
				static long getUnsignedDivision(long dividend, int divisor);

			};
		}
	}
}

#endif /* UTILITIES_MATH_MANIPULATOR_H_ */
