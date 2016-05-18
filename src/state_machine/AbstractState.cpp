/**
 *  AbstractState.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/AbstractState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::AbstractState::AbstractState(mrta_vc::state_machine::MachineController* controller, std::string question, bool final_state) : controller_(controller)
{
  question_ = question;
  message_ = "";
  final_state_ = final_state;
}

/**
 * Destructor
 */
mrta_vc::state_machine::AbstractState::~AbstractState()
{
}

/**
 * 
 */
std::string mrta_vc::state_machine::AbstractState::getQuestion()
{
  return question_;
}

/**
 * 
 */
std::string mrta_vc::state_machine::AbstractState::getMessage()
{
  return message_;
}

/**
 *
 */
bool mrta_vc::state_machine::AbstractState::isFinalState()
{
  return final_state_;
}

/**
 * 
 */
mrta_vc::state_machine::MachineController* mrta_vc::state_machine::AbstractState::getController()
{
  return controller_;
}

/**
 * 
 */
ros::NodeHandle mrta_vc::state_machine::AbstractState::getNodeHandle()
{
  return controller_->getNodeHandle();
}

/**
 * 
 */
void mrta_vc::state_machine::AbstractState::setQuestion(std::string question)
{
  question_ = question;
}

/**
 * 
 */
void mrta_vc::state_machine::AbstractState::setMessage(std::string message)
{
  message_ = message;
}

/**
 *
 */
void mrta_vc::state_machine::AbstractState::process(std::string answer)
{
}

/**
 *
 */
void mrta_vc::state_machine::AbstractState::next(std::string answer)
{
}

std::vector<std::string> mrta_vc::state_machine::AbstractState::split(std::string str, char delimiter)
{
  std::vector<std::string> internal;
  std::stringstream ss(str);
  std::string tok;

  while(std::getline(ss, tok, delimiter))
  {
    internal.push_back(tok);
  }

  return internal;
}
