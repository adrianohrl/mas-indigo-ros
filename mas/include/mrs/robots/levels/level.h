/**
 *  This header file defines the Level enumerateds.
 *
 *  Version: 1.4.0
 *  Created on: 16/09/2016
 *  Modified on: 14/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _CAPABILITY_LEVEL_ENUM_H_
#define _CAPABILITY_LEVEL_ENUM_H_

namespace mrs
{
namespace robots
{
namespace levels
{

enum LevelEnum
{
  NONE,
  LOW,
  MODERATE,
  HIGH,
  RESOURCEFUL
};
}

typedef typename levels::LevelEnum Level;
}
}

#endif // _CAPABILITY_LEVEL_ENUM_H_
