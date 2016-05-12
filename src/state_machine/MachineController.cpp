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
mrta_vc::state_machine::MachineController::MachineController(ros::NodeHandle nh) : nh_(nh)
{
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
unifei::expertinos::mrta_vc::tasks::Task mrta_vc::state_machine::MachineController::getTask()
{
    return task_;
}

/**
 *
 */
/*mrta_vc::state_machine::S8ReceiverVerificationState mrta_vc::state_machine::MachineController::getS8()
{
    return s8_;
}*/

/**
 *
 */
/*void mrta_vc::state_machine::MachineController::setNext(mrta_vc::state_machine::AbstractState state)
{
    current_ = state;
}*/
