/**
 *  This source file implements the S9FinalState class.
 *
 *	Corresponds to S9 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 15/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/s9_final_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief S9FinalState::S9FinalState
 * @param controller
 */
S9FinalState::S9FinalState(MachineController* controller)
    : AbstractState(controller, "", true)
{
}

/**
 * @brief S9FinalState::~S9FinalState
 */
S9FinalState::~S9FinalState() {}

/**
 * @brief S9FinalState::process
 * @param answer
 * @return
 */
bool S9FinalState::process(std::string answer) { return next(answer); }

/**
 * @brief S9FinalState::next
 * @param answer
 * @return
 */
bool S9FinalState::next(std::string answer) const
{
  return AbstractState::getController()->setNext(0);
}

/**
 * @brief S9FinalState::str
 * @return
 */
std::string S9FinalState::str() const { return "S9 (Final State)"; }
}
