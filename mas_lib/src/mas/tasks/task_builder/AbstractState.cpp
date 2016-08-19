/**
 *  AbstractState.cpp
 *
 *  Version: 1.2.4
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/AbstractState.h"
#include "mas/tasks/task_builder/MachineController.h"

namespace mas
{
	namespace tasks
	{
		namespace task_builder
		{

			/**
			 * Constructor
			 */
			AbstractState::AbstractState(MachineController* controller, std::string question, bool final_state)
			{
				controller_ = controller;
			  question_ = question;
			  message_ = "";
			  final_state_ = final_state;
			}

			/**
			 * Destructor
			 */
			AbstractState::~AbstractState()
			{
			}

			/**
			 * 
			 */
			std::string AbstractState::getQuestion()
			{
			  return question_;
			}

			/**
			 * 
			 */
			std::string AbstractState::getMessage()
			{
			  return message_;
			}

			/**
			 *
			 */
			bool AbstractState::isFinalState()
			{
			  return final_state_;
			}

			/**
			 * 
			 */
			MachineController* AbstractState::getController()
			{
			  return controller_;
			}

			/**
			 * 
			 */
			ros::NodeHandle AbstractState::getNodeHandle()
			{
			  return controller_->getNodeHandle();
			}

			/**
			 * 
			 */
			void AbstractState::setQuestion(std::string question)
			{
			  question_ = question;
			}

			/**
			 * 
			 */
			void AbstractState::setMessage(std::string message)
			{
			  message_ = message;
			}

			/**
			 *
			 */
			bool AbstractState::process(std::string answer)
			{
				return false;
			}

			/**
			 *
			 */
			bool AbstractState::next(std::string answer)
			{
				return false;
			}

			/**
			 *
			 */
			std::string AbstractState::toString()
			{
				return "";
			}
		
		}
	}
}
