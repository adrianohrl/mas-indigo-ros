/**
 *  S2TaskVerificationState.cpp
 *
 *	Corresponds to S2 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 23/05/2026
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S2TaskVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S2TaskVerificationState::S2TaskVerificationState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::TaskVerificationState(controller)
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S2TaskVerificationState::~S2TaskVerificationState()
{
}

/**
 *
 */
void mrta_vc::state_machine::S2TaskVerificationState::process(std::string answer)
{
  mrta_vc::state_machine::TaskVerificationState::process("send " + answer);
}

/**
 *
 */
void mrta_vc::state_machine::S2TaskVerificationState::next(std::string answer)
{
    mrta_vc::state_machine::MachineController* controller = mrta_vc::state_machine::AbstractState::getController();
    controller->setNext(controller->getS6());
}
