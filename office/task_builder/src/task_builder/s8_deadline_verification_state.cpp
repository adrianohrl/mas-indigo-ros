/**
 *  This source file implements the S8DeadlineVerificationState class.
 *
 *	Corresponds to S8 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 14/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/s8_deadline_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief S8DeadlineVerificationState::S8DeadlineVerificationState
 * @param controller
 */
S8DeadlineVerificationState::S8DeadlineVerificationState(
    MachineController* controller)
    : AbstractState(controller, "What is the deadline?")
{
}

/**
 * @brief S8DeadlineVerificationState::~S8DeadlineVerificationState
 */
S8DeadlineVerificationState::~S8DeadlineVerificationState() {}

/**
 * @brief S8DeadlineVerificationState::process
 * @param answer
 * @return
 */
bool S8DeadlineVerificationState::process(std::string answer)
{
  ros::Time deadline = utilities::TimeManipulator::getTime(answer);
  if (!utilities::TimeManipulator::isDeadline(deadline))
  {
    ros::Duration duration = utilities::TimeManipulator::getDuration(answer);
    if (!utilities::TimeManipulator::isDuration(duration))
    {
      return false;
    }
    deadline = utilities::TimeManipulator::getDeadline(duration);
  }
  AbstractState::getController()->setTaskDeadline(deadline);
  return next(answer);
}

/**
 * @brief S8DeadlineVerificationState::next
 * @param answer
 * @return
 */
bool S8DeadlineVerificationState::next(std::string answer) const
{
  return AbstractState::getController()->setNext(9);
}

/**
 * @brief S8DeadlineVerificationState::str
 * @return
 */
std::string S8DeadlineVerificationState::str() const
{
  return "S8 (Deadline Verification State)";
}
}
