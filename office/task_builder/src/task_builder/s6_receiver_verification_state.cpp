/**
 *  This source file implements the S6ReceiverVerificationState class.
 *
 *	Corresponds to S6 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 11/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/s6_receiver_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief S6ReceiverVerificationState::S6ReceiverVerificationState
 * @param controller
 */
S6ReceiverVerificationState::S6ReceiverVerificationState(
    MachineController* controller)
    : PersonVerificationState(controller, "To whom?")
{
}

/**
 * @brief S6ReceiverVerificationState::~S6ReceiverVerificationState
 */
S6ReceiverVerificationState::~S6ReceiverVerificationState() {}

/**
 * @brief S6ReceiverVerificationState::process
 * @param answer
 * @return
 */
bool S6ReceiverVerificationState::process(std::string answer)
{
  if (PersonVerificationState::process(answer))
  {
    AbstractState::getController()->setTaskReceiver(
        PersonVerificationState::getPerson());
    return next(answer);
  }
  return false;
}

/**
 * @brief S6ReceiverVerificationState::next
 * @param answer
 * @return
 */
bool S6ReceiverVerificationState::next(std::string answer) const
{
  return AbstractState::getController()->setNext(7);
}

/**
 * @brief S6ReceiverVerificationState::str
 * @return
 */
std::string S6ReceiverVerificationState::str() const
{
  return "S6 (Receiver Verification State)";
}
}
