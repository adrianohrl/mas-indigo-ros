/**
 *  S6ReceiverVerificationState.cpp
 *
 *	Corresponds to S6 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
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
void mrta_vc::state_machine::S6ReceiverVerificationState::process(std::string answer)
{ 
    mrta_vc::state_machine::PersonVerificationState::process(answer);
    if (mrta_vc::state_machine::PersonVerificationState::isValid())
    {
      mrta_vc::state_machine::AbstractState::getController()->getTask().setReceiver(mrta_vc::state_machine::PersonVerificationState::getPerson());
      next(answer);
    }
}

/**
 *
 */
void mrta_vc::state_machine::S6ReceiverVerificationState::next(std::string answer)
{
    mrta_vc::state_machine::MachineController* controller = mrta_vc::state_machine::AbstractState::getController();
    controller->setNext(controller->getS7());
}
