/**
 *  TaskVerificationState.cpp
 *
 *  Version: 1.2.2
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
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
	get_task_cli_ = mrta_vc::state_machine::AbstractState::getNodeHandle().serviceClient<mas_srvs::GetTask>("/get_task");
	generate_new_id_cli_ = mrta_vc::state_machine::AbstractState::getNodeHandle().serviceClient<mas_srvs::GenerateNewId>("/generate_new_id");
}

/**
 * Destructor
 */
mrta_vc::state_machine::TaskVerificationState::~TaskVerificationState()
{
  get_task_cli_.shutdown();
	generate_new_id_cli_.shutdown();
}

/**
 * 
 */
bool mrta_vc::state_machine::TaskVerificationState::process(std::string answer)
{ 
  mas_srvs::GetTask task_srv;
	task_srv.request.name = answer;
	if (!get_task_cli_.call(task_srv))
	{
		ROS_ERROR("There is no task registered as %s!!!", task_srv.request.name.c_str());
		ROS_ERROR("%s", task_srv.response.message.c_str());
		return task_srv.response.valid;
	}
	mas_srvs::GenerateNewId generate_new_id_srv;
	generate_new_id_srv.request.type = unifei::expertinos::mrta_vc::system::EntityTypes::toCode(unifei::expertinos::mrta_vc::system::types::TASK);
	if (!generate_new_id_cli_.call(generate_new_id_srv))
	{
		ROS_ERROR("Unexpected error while generating new task id!!!");
		return false;
	}
	task_srv.response.task.id = generate_new_id_srv.response.id;
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
