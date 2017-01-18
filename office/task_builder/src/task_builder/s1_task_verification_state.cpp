/**
 *  This source file implements the S1TaskVerificationState class.
 *
 *	Corresponds to S1 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.4.0
 *  Created on: 13/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/s1_task_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * Constructor
 */
S1TaskVerificationState::S1TaskVerificationState(MachineController* controller)
    : TaskVerificationState(controller)
{
}

/**
 * Destructor
 */
S1TaskVerificationState::~S1TaskVerificationState() {}

/**
 *
 */
bool S1TaskVerificationState::process(std::string answer)
{
  answer = "bring " + answer;
  if (TaskVerificationState::process(answer))
  {
    AbstractState::getController()->setTaskReceiver(
        AbstractState::getController()->getUser());
    return next(answer);
  }
  return false;
}

/**
 *
 */
bool S1TaskVerificationState::next(std::string answer) const
{
  return AbstractState::getController()->setNext(4);
}

/**
 *
 */
std::string S1TaskVerificationState::str() const
{
  return "S1 (Task Verification State)";
}
}
