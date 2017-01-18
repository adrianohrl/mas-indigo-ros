/**
 *  This source file implements the AllocationStates helper class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/allocation_states.h"

namespace mas
{
namespace tasks
{

/**
 * @brief AllocationStates::toEnumerated
 * @param code
 * @return
 */
states::AllocationStateEnum AllocationStates::toEnumerated(int code)
{
  states::AllocationStateEnum enumerated;
  switch (code)
  {
  case 0:
    enumerated = states::NOT_ALLOCATED;
    break;
  case 1:
    enumerated = states::ALLOCATED;
    break;
  case 2:
    enumerated = states::DISPATCHED;
    break;
  case 3:
    enumerated = states::EXECUTING;
    break;
  case 4:
    enumerated = states::SUCCEEDED;
    break;
  case 5:
    enumerated = states::ABORTED;
    break;
  case 6:
    enumerated = states::CANCELLED;
    break;
  default:
    enumerated = AllocationStates::getDefault();
  }
  return enumerated;
}

/**
 * @brief AllocationStates::toCode
 * @param enumerated
 * @return
 */
int AllocationStates::toCode(states::AllocationStateEnum enumerated)
{
  int code;
  switch (enumerated)
  {
  case states::NOT_ALLOCATED:
    code = 0;
    break;
  case states::ALLOCATED:
    code = 1;
    break;
  case states::DISPATCHED:
    code = 2;
    break;
  case states::EXECUTING:
    code = 3;
    break;
  case states::SUCCEEDED:
    code = 4;
    break;
  case states::ABORTED:
    code = 5;
    break;
  case states::CANCELLED:
    code = 6;
    break;
  default:
    code = AllocationStates::toCode(AllocationStates::getDefault());
  }
  return code;
}

/**
 * @brief AllocationStates::str
 * @param enumerated
 * @return
 */
std::string AllocationStates::str(states::AllocationStateEnum enumerated)
{
  std::string enumerated_name;
  switch (enumerated)
  {
  case states::NOT_ALLOCATED:
    enumerated_name = "NOT_ALLOCATED";
    break;
  case states::ALLOCATED:
    enumerated_name = "ALLOCATED";
    break;
  case states::DISPATCHED:
    enumerated_name = "DISPATCHED";
    break;
  case states::EXECUTING:
    enumerated_name = "EXECUTING";
    break;
  case states::SUCCEEDED:
    enumerated_name = "SUCCEEDED";
    break;
  case states::ABORTED:
    enumerated_name = "ABORTED";
    break;
  case states::CANCELLED:
    enumerated_name = "CANCELLED";
    break;
  default:
    enumerated_name = AllocationStates::str(AllocationStates::getDefault());
  }
  return enumerated_name;
}

/**
 * @brief AllocationStates::c_str
 * @param enumerated
 * @return
 */
const char *AllocationStates::c_str(AllocationStateEnum enumerated)
{
  return AllocationStates::str(enumerated).c_str();
}

/**
 * @brief AllocationStates::getDefault
 * @return
 */
states::AllocationStateEnum AllocationStates::getDefault()
{
  return states::NOT_ALLOCATED;
}

/**
 * @brief AllocationStates::getAll
 * @return
 */
std::vector<states::AllocationStateEnum> AllocationStates::getAll()
{
  std::vector<states::AllocationStateEnum> enumerateds;
  enumerateds.push_back(states::NOT_ALLOCATED);
  enumerateds.push_back(states::ALLOCATED);
  enumerateds.push_back(states::DISPATCHED);
  enumerateds.push_back(states::EXECUTING);
  enumerateds.push_back(states::SUCCEEDED);
  enumerateds.push_back(states::ABORTED);
  enumerateds.push_back(states::CANCELLED);
  return enumerateds;
}
}
}
