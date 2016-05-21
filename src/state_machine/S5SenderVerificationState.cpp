/**
 *  S5SenderVerificationState.cpp
 *
 *	Corresponds to S5 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 13/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S5SenderVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S5SenderVerificationState::S5SenderVerificationState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::SenderVerificationState(controller)
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S5SenderVerificationState::~S5SenderVerificationState()
{
}

/**
 *
 */
bool mrta_vc::state_machine::S5SenderVerificationState::process(std::string answer)
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
bool mrta_vc::state_machine::S5SenderVerificationState::next(std::string answer)
{
	mrta_vc::state_machine::AbstractState::getController()->setNextToS6();
	return true;
}

/**
 *
 */
std::string mrta_vc::state_machine::S5SenderVerificationState::toString()
{
	return "S5 (Sender Verification State)";
}
