/**
 *  S4SenderVerificationState.cpp
 *
 *	Corresponds to S4 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 13/05/2016
 *  Modified on: *********
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
void mrta_vc::state_machine::S4SenderVerificationState::next(std::string answer)
{
    mrta_vc::state_machine::MachineController* controller = mrta_vc::state_machine::AbstractState::getController();
    controller->setNext(controller->getS7());
}
