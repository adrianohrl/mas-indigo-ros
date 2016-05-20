/**
 *  TaskVerificationState.cpp
 *
 *  Version: 0.0.0.0
 *  Created on: 11/05/2016
 *  Modified on: *********
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mrta_vc/state_machine/TaskVerificationState.h"
#include "mrta_vc/state_machine/MachineController.h"

/**
 * Constructor
 */
mrta_vc::state_machine::TaskVerificationState::TaskVerificationState(mrta_vc::state_machine::MachineController* controller, std::string question) : mrta_vc::state_machine::AbstractState(controller, question)
{
	get_task_cli_ = mrta_vc::state_machine::AbstractState::getNodeHandle().serviceClient<mrta_vc::GetTask>("/get_task");
}

/**
 * Destructor
 */
mrta_vc::state_machine::TaskVerificationState::~TaskVerificationState()
{
  get_task_cli_.shutdown();
}

/**
 * 
 */
bool mrta_vc::state_machine::TaskVerificationState::process(std::string answer)
{ 
  mrta_vc::GetTask task_srv;
  task_srv.request.name = answer;
  if (!get_task_cli_.call(task_srv))
  {
    ROS_ERROR("There is no task registered as %s!!!", task_srv.request.name.c_str());
    ROS_ERROR("%s", task_srv.response.message.c_str());
		return task_srv.response.valid;
	}
	mrta_vc::state_machine::AbstractState::getController()->setTask(unifei::expertinos::mrta_vc::tasks::Task(task_srv.response.task));
	return task_srv.response.valid;
}

/**
*
*/
bool mrta_vc::state_machine::TaskVerificationState::next(std::string answer)
{
	return false;
}

/**
 *
 */
std::string mrta_vc::state_machine::TaskVerificationState::toString()
{
	return "";
}
