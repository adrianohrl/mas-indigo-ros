/**
 *  This source file implements the S4SenderVerificationState class.
 *
 *	Corresponds to S4 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 13/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/s4_sender_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief S4SenderVerificationState::S4SenderVerificationState
 * @param controller
 */
S4SenderVerificationState::S4SenderVerificationState(
    MachineController* controller)
    : SenderVerificationState(controller)
{
}

/**
 * @brief S4SenderVerificationState::~S4SenderVerificationState
 */
S4SenderVerificationState::~S4SenderVerificationState() {}

/**
 * @brief S4SenderVerificationState::process
 * @param answer
 * @return
 */
bool S4SenderVerificationState::process(std::string answer)
{
  if (SenderVerificationState::process(answer))
  {
    return next(answer);
  }
  return false;
}

/**
 * @brief S4SenderVerificationState::next
 * @param answer
 * @return
 */
bool S4SenderVerificationState::next(std::string answer) const
{
  return AbstractState::getController()->setNext(7);
}

/**
 * @brief S4SenderVerificationState::str
 * @return
 */
std::string S4SenderVerificationState::str() const
{
  return "S4 (Sender Verification State)";
}
}
