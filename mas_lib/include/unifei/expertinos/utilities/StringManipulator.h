/**
 *  StringManipulator.h
 *
 *  Version: 1.2.2
 *  Created on: 16/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef UTILITIES_STRING_MANIPULATOR_H_
#define UTILITIES_STRING_MANIPULATOR_H_

#include <sstream>
#include <string>
#include <vector>

namespace unifei
{
	namespace expertinos
	{
		namespace utilities
		{
			class StringManipulator
			{

			public:
				static std::vector<std::string> split(std::string str, char delimiter);

			};
		}
	}
}		

#endif /* UTILITIES_STRING_MANIPULATOR_H_ */
