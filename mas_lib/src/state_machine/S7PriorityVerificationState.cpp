/**
 *  S7PriorityVerificationState.cpp
 *
 *	Corresponds to S7 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.2
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
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
bool mrta_vc::state_machine::S7PriorityVerificationState::process(std::string answer)
{
  if (unifei::expertinos::mrta_vc::tasks::TaskPriorities::isValid(answer))
  {
		mrta_vc::state_machine::AbstractState::getController()->setTaskPriority(unifei::expertinos::mrta_vc::tasks::TaskPriorities::toEnumerated(answer));
		return next(answer);
  }
	return false;
}

/**
 *
 */
bool mrta_vc::state_machine::S7PriorityVerificationState::next(std::string answer)
{
	mrta_vc::state_machine::AbstractState::getController()->setNextToS8();
	return true;
}

/**
 *
 */
std::string mrta_vc::state_machine::S7PriorityVerificationState::toString()
{
	return "S7 (Priority Verification State)";
}
