/**
 *  S8DeadlineVerificationState.cpp
 *
 *	Corresponds to S8 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 0.0.0.0
 *  Created on: 14/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/S8DeadlineVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::S8DeadlineVerificationState::S8DeadlineVerificationState(mrta_vc::state_machine::MachineController* controller) : mrta_vc::state_machine::AbstractState(controller, "What is the deadline?")
{
}

/**
 * Destructor
 */
mrta_vc::state_machine::S8DeadlineVerificationState::~S8DeadlineVerificationState()
{
}

/**
 *
 */
bool mrta_vc::state_machine::S8DeadlineVerificationState::process(std::string answer)
{
	ros::Time deadline = unifei::expertinos::utilities::TimeManipulator::getTime(answer);
	if (!unifei::expertinos::utilities::TimeManipulator::isDeadline(deadline))
	{
		ros::Duration duration = unifei::expertinos::utilities::TimeManipulator::getDuration(answer);
		if (!unifei::expertinos::utilities::TimeManipulator::isDuration(duration))
		{
			return false;
		}
		deadline = unifei::expertinos::utilities::TimeManipulator::getDeadline(duration);
	}
	mrta_vc::state_machine::AbstractState::getController()->setTaskDeadline(deadline);
	return next(answer);
}

/**
 *
 */
bool mrta_vc::state_machine::S8DeadlineVerificationState::next(std::string answer)
{
	mrta_vc::state_machine::AbstractState::getController()->setNextToS9();
	return true;
}

/**
 *
 */
std::string mrta_vc::state_machine::S8DeadlineVerificationState::toString()
{
	return "S8 (Deadline Verification State)";
}
