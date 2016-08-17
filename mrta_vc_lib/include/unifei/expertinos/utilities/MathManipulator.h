/**
 *  MathManipulator.h
 *
 *  Version: 1.0.0.0
 *  Created on: 21/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef MATH_MANIPULATOR_H_
#define MATH_MANIPULATOR_H_

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

#endif /* MATH_MANIPULATOR_H_ */
