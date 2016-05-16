/**
 *  MachineController.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/MachineController.h"
#include "mrta_vc/state_machine/AbstractState.h"

/**
 * Constructor
 */
mrta_vc::state_machine::MachineController::MachineController(ros::NodeHandle nh) : nh_(nh), s0_(this), s1_(this), s2_(this), s3_(this), s4_(this), s5_(this), s6_(this), s7_(this), s8_(this), s9_(this)
{
  current_ = &s0_;
	changed_state_ = false;
}

/**
 * Destructor
 */
mrta_vc::state_machine::MachineController::~MachineController()
{
}

/**
 *
 */
ros::NodeHandle mrta_vc::state_machine::MachineController::getNodeHandle()
{
	return nh_;
}

/**
 *
 */
std::string mrta_vc::state_machine::MachineController::getQuestion()
{
	return current_->getQuestion();
}

/**
 *
 */
std::string mrta_vc::state_machine::MachineController::getMessage()
{
	return current_->getMessage();
}

/**
 *
 *
 */
bool mrta_vc::state_machine::MachineController::hasChangedState()
{
	return changed_state_;
}

/**
 *
 */
bool mrta_vc::state_machine::MachineController::isFinalState()
{
	return current_->isFinalState();
}

/**
 *
 */
unifei::expertinos::mrta_vc::tasks::Task mrta_vc::state_machine::MachineController::getTask()
{
	return task_;
}

/**
 *
 */
mrta_vc::state_machine::S0InitialState mrta_vc::state_machine::MachineController::getS0()
{
	return s0_;
}

/**
 *
 */
mrta_vc::state_machine::S1TaskVerificationState mrta_vc::state_machine::MachineController::getS1()
{
	return s1_;
}

/**
 *
 */
mrta_vc::state_machine::S2TaskVerificationState mrta_vc::state_machine::MachineController::getS2()
{
	return s2_;
}

/**
 *
 */
mrta_vc::state_machine::S3TaskVerificationState mrta_vc::state_machine::MachineController::getS3()
{
	return s3_;
}

/**
 *
 */
mrta_vc::state_machine::S4SenderVerificationState mrta_vc::state_machine::MachineController::getS4()
{
	return s4_;
}

/**
 *
 */
mrta_vc::state_machine::S5SenderVerificationState mrta_vc::state_machine::MachineController::getS5()
{
	return s5_;
}

/**
 *
 */
mrta_vc::state_machine::S6ReceiverVerificationState mrta_vc::state_machine::MachineController::getS6()
{
	return s6_;
}

/**
 *
 */
mrta_vc::state_machine::S7PriorityVerificationState mrta_vc::state_machine::MachineController::getS7()
{
	return s7_;
}

/**
 *
 */
mrta_vc::state_machine::S8DeadlineVerificationState mrta_vc::state_machine::MachineController::getS8()
{
	return s8_;
}

/**
 *
 */
mrta_vc::state_machine::S9FinalState mrta_vc::state_machine::MachineController::getS9()
{
	return s9_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setTask(unifei::expertinos::mrta_vc::tasks::Task task)
{
	task_ = task;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNext(mrta_vc::state_machine::AbstractState state)
{
	current_ = &state;
	changed_state_ = true;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::process(std::string answer)
{
	changed_state_ = false;
	if (answer == "cancel" || answer == "abort")
	{
		reset();
		return;
	}
	current_->process(answer);
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::reset()
{
	setNext(s0_);
	task_ = unifei::expertinos::mrta_vc::tasks::Task();
}
