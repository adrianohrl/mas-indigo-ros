/**
 *  PersonVerificationState.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/PersonVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::PersonVerificationState::PersonVerificationState(mrta_vc::state_machine::MachineController* controller, std::string question) : mrta_vc::state_machine::AbstractState(controller, question)
{
	ros::NodeHandle nh = mrta_vc::state_machine::AbstractState::getNodeHandle();
	get_person_cli_ = nh.serviceClient<mrta_vc::GetPerson>("/get_person");
  valid_ = false;
}

/**
 * Destructor
 */
mrta_vc::state_machine::PersonVerificationState::~PersonVerificationState()
{
	get_person_cli_.shutdown();
}

/**
 * 
 */
unifei::expertinos::mrta_vc::agents::Person mrta_vc::state_machine::PersonVerificationState::getPerson()
{
	return person_;
} 

/**
 * 
 */
void mrta_vc::state_machine::PersonVerificationState::process(std::string answer)
{ 
  valid_ = false;
	mrta_vc::GetPerson person_srv;
  person_srv.request.name = answer;
	if (!get_person_cli_.call(person_srv))
	{
    ROS_ERROR("There is no person registered as %s!!!", person_srv.request.name.c_str());
		ROS_ERROR("%s", person_srv.response.message.c_str());
		return;
	}
  person_ = unifei::expertinos::mrta_vc::agents::Person(person_srv.response.person);
  valid_ = person_srv.response.valid;
}

/**
 *
 */
bool mrta_vc::state_machine::PersonVerificationState::isValid()
{
  return valid_;
}

/**
 *
 */
void mrta_vc::state_machine::PersonVerificationState::next(std::string answer)
{
}
