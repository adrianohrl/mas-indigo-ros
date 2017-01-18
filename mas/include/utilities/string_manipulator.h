/**
 *  This header file defines the StringManipulator class.
 *
 *  Version: 1.4.0
 *  Created on: 16/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_STRING_MANIPULATOR_H_
#define _UTILITIES_STRING_MANIPULATOR_H_

#include <sstream>
#include <string>
#include <vector>

namespace utilities
{
class StringManipulator
{

public:
  static std::vector<std::string> split(std::string str, char delimiter);
};
}

#endif /* _UTILITIES_STRING_MANIPULATOR_H_ */
