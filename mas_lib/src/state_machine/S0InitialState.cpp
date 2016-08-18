/**
 *  S0InitialState.cpp
 *
 *	Corresponds to S0 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.2.2
 *  Created on: 13/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S0InitialState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S0InitialState::S0InitialState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::AbstractState(controller, "What should I do?")
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S0InitialState::~S0InitialState()
{
}

/**
 * 
 */
bool mrta_vc::state_machine::S0InitialState::process(std::string answer)
{ 
	return next(answer);
}

/**
* 
*/
bool mrta_vc::state_machine::S0InitialState::next(std::string answer)
{
	if (answer == "bring")
	{
		mrta_vc::state_machine::AbstractState::getController()->setNextToS1();
	}
	else if (answer == "send")
	{
		mrta_vc::state_machine::AbstractState::getController()->setNextToS2();
	}
	else if (answer == "take")
	{
		mrta_vc::state_machine::AbstractState::getController()->setNextToS3();
	}
	else
	{
		return false;
	}
	return true;
}

/**
 *
 */
std::string mrta_vc::state_machine::S0InitialState::toString()
{
	return "S0 (Initial State)";
}
