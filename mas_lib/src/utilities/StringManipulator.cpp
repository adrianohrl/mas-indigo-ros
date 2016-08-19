/**
 *  StringManipulator.cpp
 *
 *  Version: 1.2.4
 *  Created on: 16/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/StringManipulator.h"

namespace utilities
{
	
	/**
	 *
	 */
	std::vector<std::string> StringManipulator::split(std::string str, char delimiter)
	{
		std::vector<std::string> internal;
		std::stringstream ss(str);
		std::string tok;
		while(std::getline(ss, tok, delimiter))
		{
			internal.push_back(tok);
		}
		return internal;
	}
	
}
