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

/**
 * Constructor
 */
mrta_vc::state_machine::MachineController::MachineController(ros::NodeHandle nh) : nh_(nh), s0_(this), s1_(this), s2_(this), s3_(this), s4_(this), s5_(this), s6_(this), s7_(this), s8_(this), s9_(this)
{
	reset();
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
unifei::expertinos::mrta_vc::agents::User mrta_vc::state_machine::MachineController::getUser()
{
	return user_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNextToS0()
{
	current_ = &s0_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNextToS1()
{
	current_ = &s1_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNextToS2()
{
	current_ = &s2_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNextToS3()
{
	current_ = &s3_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNextToS4()
{
	current_ = &s4_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNextToS5()
{
	current_ = &s5_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNextToS6()
{
	current_ = &s6_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNextToS7()
{
	current_ = &s7_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNextToS8()
{
	current_ = &s8_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setNextToS9()
{
	current_ = &s9_;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setUser(unifei::expertinos::mrta_vc::agents::User user)
{
	user_ = user;
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setTask(unifei::expertinos::mrta_vc::tasks::Task task)
{
	task_ = task;
	task_.setUser(user_);
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setTaskSender(unifei::expertinos::mrta_vc::agents::Person sender)
{
	task_.setSender(sender);
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setTaskReceiver(unifei::expertinos::mrta_vc::agents::Person receiver)
{
	task_.setReceiver(receiver);
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setTaskPriority(unifei::expertinos::mrta_vc::tasks::TaskPriorityEnum priority)
{
	task_.setPriority(priority);
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setTaskDeadline(ros::Time deadline)
{
	task_.setDeadline(deadline);
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::setTaskDeadline(ros::Duration duration)
{
	task_.setDeadline(duration);
}

/**
 *
 */
bool mrta_vc::state_machine::MachineController::process(std::string answer)
{
	if (answer == "cancel" || answer == "abort")
	{
		reset();
		return true;
	}
	return current_->process(answer);
}

/**
 *
 */
void mrta_vc::state_machine::MachineController::reset()
{
	current_ = &s0_;
	task_ = unifei::expertinos::mrta_vc::tasks::Task();
}

/**
 *
 */
std::string mrta_vc::state_machine::MachineController::toString()
{
	return current_->toString();
}
