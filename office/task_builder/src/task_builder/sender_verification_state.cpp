/**
 *  This source file implements the SenderVerificationState pure abstract class.
 *
 *  Version: 1.4.0
 *  Created on: 13/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/sender_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief SenderVerificationState::SenderVerificationState
 * @param controller
 */
SenderVerificationState::SenderVerificationState(MachineController* controller)
    : PersonVerificationState(controller, "From whom?")
{
}

/**
 * @brief SenderVerificationState::~SenderVerificationState
 */
SenderVerificationState::~SenderVerificationState() {}

/**
 * @brief SenderVerificationState::process
 * @param answer
 * @return
 */
bool SenderVerificationState::process(std::string answer)
{
  if (PersonVerificationState::process(answer))
  {
    AbstractState::getController()->setTaskSender(
        PersonVerificationState::getPerson());
    return true;
  }
  return false;
}
}
