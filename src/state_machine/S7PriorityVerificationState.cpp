/**
 *  S7PriorityVerificationState.cpp
 *
 *	Corresponds to S7 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 13/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S7PriorityVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S7PriorityVerificationState::S7PriorityVerificationState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::AbstractState(controller, "How urgent?")
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S7PriorityVerificationState::~S7PriorityVerificationState()
{
}

/**
 *
 */
void mrta_vc::state_machine::S7PriorityVerificationState::process(std::string answer)
{
  mrta_vc::state_machine::S7PriorityVerificationState::process(answer);
  if (unifei::expertinos::mrta_vc::tasks::TaskPriorities::isValid(answer))
  {
    mrta_vc::state_machine::AbstractState::getController()->getTask().setPriority(unifei::expertinos::mrta_vc::tasks::TaskPriorities::toEnumerated(answer));
    next(answer);
  }
}

/**
 *
 */
void mrta_vc::state_machine::S7PriorityVerificationState::next(std::string answer)
{
  mrta_vc::state_machine::MachineController* controller = mrta_vc::state_machine::AbstractState::getController();
  controller->setNext(controller->getS8());
}
