/**
 *  S2TaskVerificationState.cpp
 *
 *	Corresponds to S2 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 23/05/2026
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S2TaskVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S2TaskVerificationState::S2TaskVerificationState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::TaskVerificationState(controller)
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S2TaskVerificationState::~S2TaskVerificationState()
{
}

/**
 *
 */
bool mrta_vc::state_machine::S2TaskVerificationState::process(std::string answer)
{
	answer = "send " + answer;
	if (mrta_vc::state_machine::TaskVerificationState::process(answer))
	{
		mrta_vc::state_machine::AbstractState::getController()->setTaskSender(mrta_vc::state_machine::AbstractState::getController()->getUser());
		return next(answer);
	}
	return false;
}

/**
 *
 */
bool mrta_vc::state_machine::S2TaskVerificationState::next(std::string answer)
{
		mrta_vc::state_machine::AbstractState::getController()->setNextToS6();
		return true;
}

/**
 *
 */
std::string mrta_vc::state_machine::S2TaskVerificationState::toString()
{
	return "S2 (Task Verification State)";
}
