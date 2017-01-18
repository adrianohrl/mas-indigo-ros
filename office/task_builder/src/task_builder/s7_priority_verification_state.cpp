/**
 *  This source file implements the S7PriorityVerificationState class.
 *
 *	Corresponds to S7 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 13/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/s7_priority_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief S7PriorityVerificationState::S7PriorityVerificationState
 * @param controller
 */
S7PriorityVerificationState::S7PriorityVerificationState(
    MachineController* controller)
    : AbstractState(controller, "How urgent?")
{
}

/**
 * @brief S7PriorityVerificationState::~S7PriorityVerificationState
 */
S7PriorityVerificationState::~S7PriorityVerificationState() {}

/**
 * @brief S7PriorityVerificationState::process
 * @param answer
 * @return
 */
bool S7PriorityVerificationState::process(std::string answer)
{
  if (mas::tasks::TaskPriorities::isValid(answer))
  {
    AbstractState::getController()->setTaskPriority(
        mas::tasks::TaskPriorities::toEnumerated(answer));
    return next(answer);
  }
  return false;
}

/**
 * @brief S7PriorityVerificationState::next
 * @param answer
 * @return
 */
bool S7PriorityVerificationState::next(std::string answer) const
{
  return AbstractState::getController()->setNext(8);
}

/**
 * @brief S7PriorityVerificationState::str
 * @return
 */
std::string S7PriorityVerificationState::str() const
{
  return "S7 (Priority Verification State)";
}
}
