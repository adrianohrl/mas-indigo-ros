/**
 *  S1TaskVerificationState.cpp
 *
 *	Corresponds to S1 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 13/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S1TaskVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S1TaskVerificationState::S1TaskVerificationState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::TaskVerificationState(controller)
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S1TaskVerificationState::~S1TaskVerificationState()
{
}

/**
 *
 */
bool mrta_vc::state_machine::S1TaskVerificationState::process(std::string answer)
{
	answer = "bring " + answer;
	if (mrta_vc::state_machine::TaskVerificationState::process(answer))
	{
		mrta_vc::state_machine::AbstractState::getController()->setTaskReceiver(mrta_vc::state_machine::AbstractState::getController()->getUser());
		return next(answer);
	}
	return false;
}

/**
 *
 */
bool mrta_vc::state_machine::S1TaskVerificationState::next(std::string answer)
{
		mrta_vc::state_machine::AbstractState::getController()->setNextToS4();
		return true;
}

/**
 *
 */
std::string mrta_vc::state_machine::S1TaskVerificationState::toString()
{
	return "S1 (Task Verification State)";
}
