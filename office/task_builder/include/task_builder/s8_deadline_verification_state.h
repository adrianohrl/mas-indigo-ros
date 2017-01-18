/**
 *  This header file defines the S8DeadlineVerificationState class.
 *
 *  Corresponds to S8 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 14/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_S8_DEADLINE_VERIFICATION_STATE_H_
#define _TASK_BUILDER_S8_DEADLINE_VERIFICATION_STATE_H_

#include <mas/tasks/task_priorities.h>
#include <utilities/time_manipulator.h>
#include "task_builder/abstract_state.h"

#define DEFAULT_DURATION 3600

namespace task_builder
{
class S8DeadlineVerificationState : public AbstractState
{
public:
  S8DeadlineVerificationState(MachineController* controller);
  virtual ~S8DeadlineVerificationState();
  virtual bool process(std::string answer);
  virtual std::string str() const;

private:
  virtual bool next(std::string answer) const;
};
}

#endif /* _TASK_BUILDER_S8_DEADLINE_VERIFICATION_STATE_H_ */
