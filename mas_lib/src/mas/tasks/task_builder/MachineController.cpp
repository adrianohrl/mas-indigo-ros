/**
 *  MachineController.cpp
 *
 *  Version: 1.2.4
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

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
			MachineController::MachineController(ros::NodeHandle nh) : nh_(nh), s0_(this), s1_(this), s2_(this), s3_(this), s4_(this), s5_(this), s6_(this), s7_(this), s8_(this), s9_(this)
			{
				reset();
			}

			/**
			 * Destructor
			 */
			MachineController::~MachineController()
			{
			}

			/**
			 *
			 */
			ros::NodeHandle MachineController::getNodeHandle()
			{
				return nh_;
			}

			/**
			 *
			 */
			std::string MachineController::getQuestion()
			{
				return current_->getQuestion();
			}

			/**
			 *
			 */
			std::string MachineController::getMessage()
			{
				return current_->getMessage();
			}

			/**
			 *
			 */
			bool MachineController::isFinalState()
			{
				return current_->isFinalState();
			}

			/**
			 *
			 */
			tasks::Task MachineController::getTask()
			{
				return task_;
			}

			/**
			 *
			 */
			agents::User MachineController::getUser()
			{
				return user_;
			}

			/**
			 *
			 */
			void MachineController::setNextToS0()
			{
				current_ = &s0_;
			}

			/**
			 *
			 */
			void MachineController::setNextToS1()
			{
				current_ = &s1_;
			}

			/**
			 *
			 */
			void MachineController::setNextToS2()
			{
				current_ = &s2_;
			}

			/**
			 *
			 */
			void MachineController::setNextToS3()
			{
				current_ = &s3_;
			}

			/**
			 *
			 */
			void MachineController::setNextToS4()
			{
				current_ = &s4_;
			}

			/**
			 *
			 */
			void MachineController::setNextToS5()
			{
				current_ = &s5_;
			}

			/**
			 *
			 */
			void MachineController::setNextToS6()
			{
				current_ = &s6_;
			}

			/**
			 *
			 */
			void MachineController::setNextToS7()
			{
				current_ = &s7_;
			}

			/**
			 *
			 */
			void MachineController::setNextToS8()
			{
				current_ = &s8_;
			}

			/**
			 *
			 */
			void MachineController::setNextToS9()
			{
				current_ = &s9_;
			}

			/**
			 *
			 */
			void MachineController::setUser(agents::User user)
			{
				user_ = user;
			}

			/**
			 *
			 */
			void MachineController::setTask(tasks::Task task)
			{
				task_ = task;
				task_.setUser(user_);
			}

			/**
			 *
			 */
			void MachineController::setTaskSender(agents::Person sender)
			{
				task_.setSender(sender);
			}

			/**
			 *
			 */
			void MachineController::setTaskReceiver(agents::Person receiver)
			{
				task_.setReceiver(receiver);
			}

			/**
			 *
			 */
			void MachineController::setTaskPriority(tasks::TaskPriorityEnum priority)
			{
				task_.setPriority(priority);
			}

			/**
			 *
			 */
			void MachineController::setTaskDeadline(ros::Time deadline)
			{
				task_.setDeadline(deadline);
			}

			/**
			 *
			 */
			void MachineController::setTaskDeadline(ros::Duration duration)
			{
				task_.setDeadline(duration);
			}

			/**
			 *
			 */
			bool MachineController::process(std::string answer)
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
			void MachineController::reset()
			{
				current_ = &s0_;
				task_ = tasks::Task();
			}

			/**
			 *
			 */
			std::string MachineController::toString()
			{
				return current_->toString();
			}
			
		}
	}
}
