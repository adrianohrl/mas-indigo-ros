/**
 *  This source file implements the S2TaskVerificationState class.
 *
 *	Corresponds to S2 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 13/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/s2_task_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief S2TaskVerificationState::S2TaskVerificationState
 * @param controller
 */
S2TaskVerificationState::S2TaskVerificationState(MachineController* controller)
    : TaskVerificationState(controller)
{
}

/**
 * @brief S2TaskVerificationState::~S2TaskVerificationState
 */
S2TaskVerificationState::~S2TaskVerificationState() {}

/**
 * @brief S2TaskVerificationState::process
 * @param answer
 * @return
 */
bool S2TaskVerificationState::process(std::string answer)
{
  answer = "send " + answer;
  if (TaskVerificationState::process(answer))
  {
    AbstractState::getController()->setTaskSender(
        AbstractState::getController()->getUser());
    return next(answer);
  }
  return false;
}

/**
 * @brief S2TaskVerificationState::next
 * @param answer
 * @return
 */
bool S2TaskVerificationState::next(std::string answer) const
{
  return AbstractState::getController()->setNext(6);
}

/**
 * @brief S2TaskVerificationState::str
 * @return
 */
std::string S2TaskVerificationState::str() const
{
  return "S2 (Task Verification State)";
}
}
