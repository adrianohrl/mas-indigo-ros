/**
 *  PersonVerificationState.cpp
 *
 *  Version: 1.2.4
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/PersonVerificationState.h"
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
			PersonVerificationState::PersonVerificationState(MachineController* controller, std::string question) : AbstractState(controller, question)
			{
				get_person_cli_ = AbstractState::getNodeHandle().serviceClient<mas_srvs::GetPerson>("/get_person");
			}

			/**
			 * Destructor
			 */
			PersonVerificationState::~PersonVerificationState()
			{
				get_person_cli_.shutdown();
			}

			/**
			 * 
			 */
			agents::Person PersonVerificationState::getPerson()
			{
				return person_;
			} 

			/**
			 * 
			 */
			bool PersonVerificationState::process(std::string answer)
			{ 
				mas_srvs::GetPerson person_srv;
			  person_srv.request.name = answer;
				if (!get_person_cli_.call(person_srv))
				{
			    ROS_ERROR("There is no person registered as %s!!!", person_srv.request.name.c_str());
					ROS_ERROR("%s", person_srv.response.message.c_str());
					return person_srv.response.valid;
				}
			  person_ = agents::Person(person_srv.response.person);
				return person_srv.response.valid;
			}

			/**
			 *
			 */
			bool PersonVerificationState::next(std::string answer)
			{
				return false;
			}

			/**
			 *
			 */
			std::string PersonVerificationState::toString()
			{
				return "";
			}
			
		}
	}
}
