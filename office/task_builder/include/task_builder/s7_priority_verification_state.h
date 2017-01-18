/**
 *  This header file defines the S7PriorityVerificationState class.
 *
 *  Corresponds to S7 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 13/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_S7_PRIORITY_VERIFICATION_STATE_H_
#define _TASK_BUILDER_S7_PRIORITY_VERIFICATION_STATE_H_

#include <mas/tasks/task_priorities.h>
#include "task_builder/abstract_state.h"

namespace task_builder
{
class S7PriorityVerificationState : public AbstractState
{
public:
  S7PriorityVerificationState(MachineController* controller);
  virtual ~S7PriorityVerificationState();
  virtual bool process(std::string answer);
  virtual std::string str() const;

private:
  virtual bool next(std::string answer) const;
};
}

#endif /* _TASK_BUILDER_S7_PRIORITY_VERIFICATION_STATE_H_ */
