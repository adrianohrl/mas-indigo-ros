/**
 *  S9FinalState.cpp
 *
 *	Corresponds to S9 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 15/05/2016
 *  Modified on: *********
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
void mrta_vc::state_machine::S9FinalState::process(std::string answer = "")
{
  next(answer);
}

/**
 *
 */
void mrta_vc::state_machine::S9FinalState::next(std::string answer = "")
{
  mrta_vc::state_machine::MachineController* controller = mrta_vc::state_machine::AbstractState::getController();
  controller->setNext(controller->getS0());
}
