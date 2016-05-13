/**
 *  S0InitialState.cpp
 *
 *	Corresponds to S0 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 13/05/2016
 *  Modified on: *********
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
void mrta_vc::state_machine::S0InitialState::process(std::string answer)
{ 
  next(answer);
}

/**
* 
*/
void mrta_vc::state_machine::S0InitialState::next(std::string answer)
{
	mrta_vc::state_machine::MachineController* controller = mrta_vc::state_machine::AbstractState::getController();
	if (answer == "bring")
	{
		controller->setNext(controller->getS1());
	}
	else if (answer == "send")
	{
		controller->setNext(controller->getS2());
	}
	else if (answer == "take")
	{
		controller->setNext(controller->getS3());
	}
}
