/**
 *  This source file implements the S0InitialState class.
 *
 *	Corresponds to S0 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 13/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/s0_initial_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief S0InitialState::S0InitialState
 * @param controller
 */
S0InitialState::S0InitialState(MachineController* controller)
    : AbstractState(controller, "What should I do?")
{
}

/**
 * @brief S0InitialState::~S0InitialState
 */
S0InitialState::~S0InitialState() {}

/**
 * @brief S0InitialState::process
 * @param answer
 * @return
 */
bool S0InitialState::process(std::string answer) { return next(answer); }

/**
 * @brief S0InitialState::next
 * @param answer
 * @return
 */
bool S0InitialState::next(std::string answer) const
{
  if (answer == "bring")
  {
    return AbstractState::getController()->setNext(1);
  }
  else if (answer == "send")
  {
    return AbstractState::getController()->setNext(2);
  }
  else if (answer == "take")
  {
    return AbstractState::getController()->setNext(3);
  }
  return false;
}

/**
 * @brief S0InitialState::str
 * @return
 */
std::string S0InitialState::str() const { return "S0 (Initial State)"; }
}
