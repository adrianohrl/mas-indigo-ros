/**
 *  This source file implements the PersonVerificationState pure abstract class.
 *
 *  Version: 1.4.0
 *  Created on: 11/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/person_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief PersonVerificationState::PersonVerificationState
 * @param controller
 * @param question
 */
PersonVerificationState::PersonVerificationState(MachineController* controller,
                                                 std::string question)
    : AbstractState(controller, question)
{
  get_person_cli_ =
      AbstractState::getNodeHandle()->serviceClient<mas_srvs::GetPerson>(
          "/get_person");
}

/**
 * @brief PersonVerificationState::~PersonVerificationState
 */
PersonVerificationState::~PersonVerificationState()
{
  get_person_cli_.shutdown();
}

/**
 * @brief PersonVerificationState::getPerson
 * @return
 */
mas::agents::Person* PersonVerificationState::getPerson() const { return person_; }

/**
 * @brief PersonVerificationState::process
 * @param answer
 * @return
 */
bool PersonVerificationState::process(std::string answer)
{
  mas_srvs::GetPerson person_srv;
  person_srv.request.name = answer;
  if (!get_person_cli_.call(person_srv))
  {
    ROS_ERROR("There is no person registered as %s!!!",
              person_srv.request.name.c_str());
    return person_srv.response.valid;
  }
  person_ = new mas::agents::Person(person_srv.response.person);
  return person_srv.response.valid;
}
}
