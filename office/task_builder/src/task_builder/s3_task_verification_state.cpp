/**
 *  This source file implements the S3TaskVerificationState class.
 *
 *	Corresponds to S3 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 13/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/s3_task_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief S3TaskVerificationState::S3TaskVerificationState
 * @param controller
 */
S3TaskVerificationState::S3TaskVerificationState(MachineController* controller)
    : TaskVerificationState(controller)
{
}

/**
 * @brief S3TaskVerificationState::~S3TaskVerificationState
 */
S3TaskVerificationState::~S3TaskVerificationState() {}

/**
 * @brief S3TaskVerificationState::process
 * @param answer
 * @return
 */
bool S3TaskVerificationState::process(std::string answer)
{
  answer = "take " + answer;
  if (TaskVerificationState::process(answer))
  {
    return next(answer);
  }
  return false;
}

/**
 * @brief S3TaskVerificationState::next
 * @param answer
 * @return
 */
bool S3TaskVerificationState::next(std::string answer) const
{
  return AbstractState::getController()->setNext(5);
}

/**
 * @brief S3TaskVerificationState::str
 * @return
 */
std::string S3TaskVerificationState::str() const
{
  return "S3 (Task Verification State)";
}
}
