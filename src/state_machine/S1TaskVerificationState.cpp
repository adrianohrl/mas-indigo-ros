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
void mrta_vc::state_machine::S1TaskVerificationState::process(std::string answer)
{
  mrta_vc::state_machine::TaskVerificationState::process("bring " + answer);
}

/**
 *
 */
void mrta_vc::state_machine::S1TaskVerificationState::next(std::string answer)
{
    mrta_vc::state_machine::MachineController* controller = mrta_vc::state_machine::AbstractState::getController();
    controller->setNext(controller->getS4());
}
