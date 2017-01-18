/**
 *  This header file defines the AllocationSatisfactionEnum enumerateds and the
 *AllocationSatisfactions helper class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASKS_ALLOCATION_SATISFACTIONS_H_
#define _TASKS_ALLOCATION_SATISFACTIONS_H_

#include <string>
#include <vector>

namespace mas
{
namespace tasks
{
namespace satisfactions
{
enum AllocationSatisfactionEnum
{
  NONE,
  VERY_DISSATISFIED,
  HORRIBLE,
  DISSATISFIED,
  VERY_BAD,
  SOMEWHAT_DISSATISFIED,
  BAD,
  NEITHER_DISSATISFIED_NOR_SATISFIED,
  FAIR_ENOUGH,
  SOMEWHAT_SATISFIED,
  GOOD,
  SATISFIED,
  VERY_GOOD,
  VERY_SATISFIED,
  EXCELLENT
};
}

typedef satisfactions::AllocationSatisfactionEnum AllocationSatisfactionEnum;

class AllocationSatisfactions
{
public:
  static AllocationSatisfactionEnum toEnumerated(int code);
  static int toCode(AllocationSatisfactionEnum enumerated);
  static std::string str(AllocationSatisfactionEnum enumerated);
  static const char* c_str(AllocationSatisfactionEnum enumerated);
  static AllocationSatisfactionEnum getDefault();
  static bool isValid(AllocationSatisfactionEnum enumerated);
  static std::vector<AllocationSatisfactionEnum> getAll();
};
}
}

#endif /* _TASKS_ALLOCATION_SATISFACTIONS_H_ */
