/**
 *  This source file implements the TaskVerificationState pure abstract class.
 *
 *  Version: 1.4.0
 *  Created on: 11/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "task_builder/task_verification_state.h"
#include "task_builder/machine_controller.h"

namespace task_builder
{

/**
 * @brief TaskVerificationState::TaskVerificationState
 * @param controller
 * @param question
 */
TaskVerificationState::TaskVerificationState(MachineController* controller,
                                             std::string question)
    : AbstractState(controller, question)
{
  get_task_cli_ =
      AbstractState::getNodeHandle()->serviceClient<mas_srvs::GetTask>(
          "/get_task");
  generate_new_id_cli_ =
      AbstractState::getNodeHandle()->serviceClient<mas_srvs::GenerateNewId>(
          "/generate_new_id");
}

/**
 * @brief TaskVerificationState::~TaskVerificationState
 */
TaskVerificationState::~TaskVerificationState()
{
  get_task_cli_.shutdown();
  generate_new_id_cli_.shutdown();
}

/**
 * @brief TaskVerificationState::process
 * @param answer
 * @return
 */
bool TaskVerificationState::process(std::string answer)
{
  mas_srvs::GetTask task_srv;
  task_srv.request.name = answer;
  if (!get_task_cli_.call(task_srv))
  {
    ROS_ERROR("There is no task registered as %s!!!",
              task_srv.request.name.c_str());
    ROS_ERROR("%s", task_srv.response.message.c_str());
    return task_srv.response.valid;
  }
  mas_srvs::GenerateNewId generate_new_id_srv;
  generate_new_id_srv.request.type =
      mas::database::EntityTypes::toCode(mas::database::types::TASK);
  if (!generate_new_id_cli_.call(generate_new_id_srv))
  {
    ROS_ERROR("Unexpected error while generating new task id!!!");
    return false;
  }
  task_srv.response.task.id = generate_new_id_srv.response.id;
  AbstractState::getController()->setTask(new mas::tasks::Task(task_srv.response.task));
  return task_srv.response.valid;
}
}
