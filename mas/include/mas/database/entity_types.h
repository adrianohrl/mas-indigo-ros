/**
 *  This header file defines the EntityTypes class.
 *
 *  Version: 1.4.0
 *  Created on: 08/06/2016
 *  Modified on: 14/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _SYSTEM_ENTITY_TYPES_H_
#define _SYSTEM_ENTITY_TYPES_H_

#include <string>
#include <vector>

namespace mas
{
namespace database
{
namespace types
{
enum EntityTypeEnum
{
  AGENT,
  ALLOCATION,
  PLACE,
  RESOURCE,
  SKILL,
  TASK
};
}

typedef types::EntityTypeEnum EntityTypeEnum;

class EntityTypes
{

public:
  static EntityTypeEnum toEnumerated(int code);
  static EntityTypeEnum toEnumerated(std::string name);
  static int toCode(EntityTypeEnum enumerated);
  static std::string toString(EntityTypeEnum enumerated);
  static EntityTypeEnum getDefault();
  static std::vector<EntityTypeEnum> getAll();
};
}
}

#endif /* _SYSTEM_ENTITY_TYPES_H_ */
