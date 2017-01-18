/**
 *  This header file defines the S5SenderVerificationState class.
 *
 *  Corresponds to S5 State in the Task Builder State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 11/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_S5_SENDER_VERIFICATION_STATE_H_
#define _TASK_BUILDER_S5_SENDER_VERIFICATION_STATE_H_

#include "task_builder/sender_verification_state.h"

namespace task_builder
{
class S5SenderVerificationState : public SenderVerificationState
{
public:
  S5SenderVerificationState(MachineController* controller);
  virtual ~S5SenderVerificationState();
  virtual bool process(std::string answer);
  virtual std::string str() const;

private:
  virtual bool next(std::string answer) const;
};
}

#endif /* _TASK_BUILDER_S5_SENDER_VERIFICATION_STATE_H_ */
