/**
 *  S6ReceiverVerificationState.cpp
 *
 *	Corresponds to S6 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.2
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S6ReceiverVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S6ReceiverVerificationState::S6ReceiverVerificationState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::PersonVerificationState(controller, "To whom?")
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S6ReceiverVerificationState::~S6ReceiverVerificationState()
{
}

/**
 * 
 */
bool mrta_vc::state_machine::S6ReceiverVerificationState::process(std::string answer)
{ 
	if (mrta_vc::state_machine::PersonVerificationState::process(answer))
	{
		mrta_vc::state_machine::AbstractState::getController()->setTaskReceiver(mrta_vc::state_machine::PersonVerificationState::getPerson());
		return next(answer);
	}
	return false;
}

/**
 *
 */
bool mrta_vc::state_machine::S6ReceiverVerificationState::next(std::string answer)
{
	mrta_vc::state_machine::AbstractState::getController()->setNextToS7();
	return true;
}

/**
 *
 */
std::string mrta_vc::state_machine::S6ReceiverVerificationState::toString()
{
	return "S6 (Receiver Verification State)";
}
