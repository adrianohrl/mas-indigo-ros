/**
 *  This source file implements the S5SenderVerificationState class.
 *
 *	Corresponds to S5 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 13/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/s5_sender_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief S5SenderVerificationState::S5SenderVerificationState
 * @param controller
 */
S5SenderVerificationState::S5SenderVerificationState(
    MachineController* controller)
    : SenderVerificationState(controller)
{
}

/**
 * @brief S5SenderVerificationState::~S5SenderVerificationState
 */
S5SenderVerificationState::~S5SenderVerificationState() {}

/**
 * @brief S5SenderVerificationState::process
 * @param answer
 * @return
 */
bool S5SenderVerificationState::process(std::string answer)
{
  if (SenderVerificationState::process(answer))
  {
    return next(answer);
  }
  return false;
}

/**
 * @brief S5SenderVerificationState::next
 * @param answer
 * @return
 */
bool S5SenderVerificationState::next(std::string answer) const
{
  return AbstractState::getController()->setNext(6);
}

/**
 * @brief S5SenderVerificationState::str
 * @return
 */
std::string S5SenderVerificationState::str() const
{
  return "S5 (Sender Verification State)";
}
}
