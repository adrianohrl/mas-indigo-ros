/**
 *  S4SenderVerificationState.cpp
 *
 *	Corresponds to S4 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.2
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S4SenderVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S4SenderVerificationState::S4SenderVerificationState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::SenderVerificationState(controller)
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S4SenderVerificationState::~S4SenderVerificationState()
{
}

/**
 *
 */
bool mrta_vc::state_machine::S4SenderVerificationState::process(std::string answer)
{
	if (mrta_vc::state_machine::SenderVerificationState::process(answer))
	{
		return next(answer);
	}
	return false;
}

/**
 *
 */
bool mrta_vc::state_machine::S4SenderVerificationState::next(std::string answer)
{
	mrta_vc::state_machine::AbstractState::getController()->setNextToS7();
	return true;
}

/**
 *
 */
std::string mrta_vc::state_machine::S4SenderVerificationState::toString()
{
	return "S4 (Sender Verification State)";
}
