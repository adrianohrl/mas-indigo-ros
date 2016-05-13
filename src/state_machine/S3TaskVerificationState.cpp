/**
 *  S3TaskVerificationState.cpp
 *
 *	Corresponds to S3 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 33/05/2036
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S3TaskVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S3TaskVerificationState::S3TaskVerificationState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::TaskVerificationState(controller)
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S3TaskVerificationState::~S3TaskVerificationState()
{
}

/**
 *
 */
void mrta_vc::state_machine::S3TaskVerificationState::next()
{
    mrta_vc::state_machine::MachineController* controller = mrta_vc::state_machine::AbstractState::getController();
    controller->setNext(controller->getS5());
}
