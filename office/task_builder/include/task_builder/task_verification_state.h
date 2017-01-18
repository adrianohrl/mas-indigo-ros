/**
 *  This header file defines the TaskVerificationState pure abstract class.
 *
 *  Version: 1.4.0
 *  Created on: 11/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_TASK_VERIFICATION_STATE_H_
#define _TASK_BUILDER_TASK_VERIFICATION_STATE_H_

#include <mas_srvs/GetTask.h>
#include <mas_srvs/GenerateNewId.h>
#include <mas/database/entity_types.h>
#include <mas/tasks/task.h>
#include "task_builder/abstract_state.h"

namespace task_builder
{
class TaskVerificationState : public AbstractState
{
public:
  virtual ~TaskVerificationState();
  virtual bool process(std::string answer);
  virtual std::string str() const = 0;

protected:
  TaskVerificationState(MachineController* controller,
                        std::string question = "What?");

private:
  ros::ServiceClient get_task_cli_;
  ros::ServiceClient generate_new_id_cli_;
  virtual bool next(std::string answer) const = 0;
};
}

#endif /* _TASK_BUILDER_TASK_VERIFICATION_STATE_H_ */
