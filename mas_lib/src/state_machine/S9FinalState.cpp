/**
 *  S9FinalState.cpp
 *
 *	Corresponds to S9 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.2
 *  Created on: 15/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S9FinalState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S9FinalState::S9FinalState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::AbstractState(controller, "", true)
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S9FinalState::~S9FinalState()
{
}

/**
 *
 */
bool mrta_vc::state_machine::S9FinalState::process(std::string answer)
{
	return next(answer);
}

/**
 *
 */
bool mrta_vc::state_machine::S9FinalState::next(std::string answer)
{
	mrta_vc::state_machine::AbstractState::getController()->setNextToS0();
	return true;
}

/**
 *
 */
std::string mrta_vc::state_machine::S9FinalState::toString()
{
	return "S9 (Final State)";
}
