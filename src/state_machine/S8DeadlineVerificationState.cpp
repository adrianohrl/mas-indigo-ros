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
  if (isDeadline(answer))
  {
		mrta_vc::state_machine::AbstractState::getController()->setTaskDeadline(getDeadline(answer));
  } 
  else if (isDuration(answer))
  {
		mrta_vc::state_machine::AbstractState::getController()->setTaskDeadline(getDuration(answer));
  }
  else if (answer == "")
	{
		mrta_vc::state_machine::AbstractState::getController()->setTaskDeadline(ros::Time::now() + ros::Duration(DEFAULT_DURATION));
  }
	else
	{
		return false;
	}
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
bool mrta_vc::state_machine::S8DeadlineVerificationState::isDeadline(std::string answer)
{
	return false;//unifei::expertinos::mrta_vc::utilities::StringManipulator::isTimestampValid(answer);
}

/**
 *
 */
bool mrta_vc::state_machine::S8DeadlineVerificationState::isDuration(std::string answer)
{
	return false;//unifei::expertinos::mrta_vc::utilities::StringManipulator::isDurationValid(answer);
}

/**
 *
 */
ros::Time mrta_vc::state_machine::S8DeadlineVerificationState::getDeadline(std::string answer)
{
	return ros::Time::now();//unifei::expertinos::mrta_vc::utilities::StringManipulator::getTimestamp(answer);
}

/**
 *
 */
ros::Duration mrta_vc::state_machine::S8DeadlineVerificationState::getDuration(std::string answer)
{
	return ros::Duration(DEFAULT_DURATION);//unifei::expertinos::mrta_vc::utilities::StringManipulator::getDuration(answer);
}

/**
 *
 */
std::string mrta_vc::state_machine::S8DeadlineVerificationState::toString()
{
	return "S8 (Deadline Verification State)";
}
