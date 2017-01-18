/**
 *  This header file defines the S4SenderVerificationState class.
 *
 *  Corresponds to S4 State in the Task Builder State Machine Model Diagram
 *
 *  Version: 1.4.0
 *  Created on: 13/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_S4_SENDER_VERIFICATION_STATE_H_
#define _TASK_BUILDER_S4_SENDER_VERIFICATION_STATE_H_

#include "task_builder/sender_verification_state.h"

namespace task_builder
{
class S4SenderVerificationState : public SenderVerificationState
{
public:
  S4SenderVerificationState(MachineController* controller);
  virtual ~S4SenderVerificationState();
  virtual bool process(std::string answer);
  virtual std::string str() const;

private:
  virtual bool next(std::string answer) const;
};
}

#endif /* TASK_BUILDER_S4_SENDER_VERIFICATION_STATE_H_ */
