/**
 *  This source file implements the AbstractState pure abstract class.
 *
 *  Version: 1.4.0
 *  Created on: 11/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/abstract_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief AbstractState::AbstractState
 * @param controller
 * @param question
 * @param final_state
 */
AbstractState::AbstractState(MachineController* controller,
                             std::string question, bool final_state) :
  controller_(controller),
  question_(question),
  message_(""),
  final_state_(final_state)
{
}

/**
 * @brief AbstractState::~AbstractState
 */
AbstractState::~AbstractState() {}

/**
 * @brief AbstractState::getQuestion
 * @return
 */
std::string AbstractState::getQuestion() const { return question_; }

/**
 * @brief AbstractState::getMessage
 * @return
 */
std::string AbstractState::getMessage() const { return message_; }

/**
 * @brief AbstractState::isFinalState
 * @return
 */
bool AbstractState::isFinalState() const { return final_state_; }

/**
 * @brief AbstractState::getController
 * @return
 */
MachineController* AbstractState::getController() const { return controller_; }

/**
 * @brief AbstractState::getNodeHandle
 * @return
 */
ros::NodeHandle* AbstractState::getNodeHandle() const
{
  return controller_->getNodeHandle();
}

/**
 * @brief AbstractState::setQuestion
 * @param question
 */
void AbstractState::setQuestion(std::string question) { question_ = question; }

/**
 * @brief AbstractState::setMessage
 * @param message
 */
void AbstractState::setMessage(std::string message) { message_ = message; }

/**
 * @brief AbstractState::c_str
 * @return
 */
const char *AbstractState::c_str() const
{
  return str().c_str();
}
}
