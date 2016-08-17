/**
 *  StringManipulator.h
 *
 *  Version: 1.0.0.0
 *  Created on: 16/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef STRING_MANIPULATOR_H_
#define STRING_MANIPULATOR_H_

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

#endif /* STRING_MANIPULATOR_H_ */
