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
bool mrta_vc::state_machine::S3TaskVerificationState::process(std::string answer)
{
	answer = "take " + answer;
	if (mrta_vc::state_machine::TaskVerificationState::process(answer))
	{
		return next(answer);
	}
	return false;
}

/**
 *
 */
bool mrta_vc::state_machine::S3TaskVerificationState::next(std::string answer)
{
		mrta_vc::state_machine::AbstractState::getController()->setNextToS5();
		return true;
}

/**
 *
 */
std::string mrta_vc::state_machine::S3TaskVerificationState::toString()
{
	return "S3 (Task Verification State)";
}
