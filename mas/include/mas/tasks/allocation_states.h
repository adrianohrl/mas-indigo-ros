/**
 *  This header file defines the AllocationStateEnum enumerateds and the
 *AllocationStates helper class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASKS_ALLOCATION_STATES_H_
#define _TASKS_ALLOCATION_STATES_H_

#include <string>
#include <vector>

namespace mas
{
namespace tasks
{
namespace states
{
enum AllocationStateEnum
{
  NOT_ALLOCATED,
  ALLOCATED,
  DISPATCHED,
  EXECUTING,
  CANCELLED,
  SUCCEEDED,
  ABORTED
};
}

typedef states::AllocationStateEnum AllocationStateEnum;

class AllocationStates
{

public:
  static AllocationStateEnum toEnumerated(int code);
  static int toCode(AllocationStateEnum enumerated);
  static std::string str(AllocationStateEnum enumerated);
  static const char* c_str(AllocationStateEnum enumerated);
  static AllocationStateEnum getDefault();
  static std::vector<AllocationStateEnum> getAll();
};
}
}

#endif /* _TASKS_ALLOCATION_STATES_H_ */
