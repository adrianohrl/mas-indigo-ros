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

/**
 * Constructor
 */
mrta_vc::state_machine::AbstractState::AbstractState(mrta_vc::state_machine::MachineController controller, std::string question) : controller_(controller)
{
  question_ = question;
  message_ = "";
  answer_ = "";
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
std::string mrta_vc::state_machine::AbstractState::getAnswer()
{
  return answer_;
}

/**
 * 
 */
mrta_vc::state_machine::MachineController mrta_vc::state_machine::AbstractState::getController()
{
  return controller_;
}

/**
 * 
 */
ros::NodeHandle mrta_vc::state_machine::AbstractState::getNodeHandle()
{
  return controller_.getNodeHandle();
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
void mrta_vc::state_machine::AbstractState::setAnswer(std::string answer)
{
  answer_ = answer;
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
bool mrta_vc::state_machine::AbstractState::isValid()
{
    return false;
}

/**
* 
*/
void mrta_vc::state_machine::AbstractState::next()
{
}
