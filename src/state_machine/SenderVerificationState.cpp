/**
 *  SenderVerificationState.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 13/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/SenderVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::SenderVerificationState::SenderVerificationState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::PersonVerificationState(controller, "To whom?")
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::SenderVerificationState::~SenderVerificationState()
{
}

/**
 *
 */
void mrta_vc::state_machine::SenderVerificationState::process(std::string answer)
{
  mrta_vc::state_machine::PersonVerificationState::process(answer);
  if (mrta_vc::state_machine::PersonVerificationState::isValid())
  {
    mrta_vc::state_machine::AbstractState::getController()->getTask().setSender(mrta_vc::state_machine::PersonVerificationState::getPerson());
    next(answer);
  }
}

/**
 *
 */
void mrta_vc::state_machine::SenderVerificationState::next(std::string answer)
{
}
