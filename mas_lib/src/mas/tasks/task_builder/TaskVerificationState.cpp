/**
 *  TaskVerificationState.cpp
 *
 *  Version: 1.2.4
 *  Created on: 11/05/2016
 *  Modified on: 17/08/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/tasks/task_builder/TaskVerificationState.h"
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
			TaskVerificationState::TaskVerificationState(MachineController* controller, std::string question) : AbstractState(controller, question)
			{
				get_task_cli_ = AbstractState::getNodeHandle().serviceClient<mas_srvs::GetTask>("/get_task");
				generate_new_id_cli_ = AbstractState::getNodeHandle().serviceClient<mas_srvs::GenerateNewId>("/generate_new_id");
			}

			/**
			 * Destructor
			 */
			TaskVerificationState::~TaskVerificationState()
			{
			  get_task_cli_.shutdown();
				generate_new_id_cli_.shutdown();
			}

			/**
			 * 
			 */
			bool TaskVerificationState::process(std::string answer)
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
				generate_new_id_srv.request.type = database::EntityTypes::toCode(database::types::TASK);
				if (!generate_new_id_cli_.call(generate_new_id_srv))
				{
					ROS_ERROR("Unexpected error while generating new task id!!!");
					return false;
				}
				task_srv.response.task.id = generate_new_id_srv.response.id;
				AbstractState::getController()->setTask(tasks::Task(task_srv.response.task));
				return task_srv.response.valid;
			}

			/**
			*
			*/
			bool TaskVerificationState::next(std::string answer)
			{
				return false;
			}

			/**
			 *
			 */
			std::string TaskVerificationState::toString()
			{
				return "";
			}
			
		}
	}
}
