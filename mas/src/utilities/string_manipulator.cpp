/**
 *  This source file implements the StringManipulator class.
 *
 *  Version: 1.4.0
 *  Created on: 16/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/string_manipulator.h"

namespace utilities
{
	
/**
 * @brief StringManipulator::split
 * @param str
 * @param delimiter
 * @return
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
