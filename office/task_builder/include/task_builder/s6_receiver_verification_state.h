/**
 *  This header file defines the S6ReceiverVerificationState class.
 *
 *  Corresponds to S6 State in the State Machine Model Diagram.
 *
 *  Version: 1.4.0
 *  Created on: 11/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_S6_RECEIVER_VERIFICATION_STATE_H_
#define _TASK_BUILDER_S6_RECEIVER_VERIFICATION_STATE_H_

#include "task_builder/person_verification_state.h"

namespace task_builder
{
class S6ReceiverVerificationState : public PersonVerificationState
{
public:
  S6ReceiverVerificationState(MachineController* controller);
  virtual ~S6ReceiverVerificationState();
  virtual bool process(std::string answer);
  virtual std::string str() const;

private:
  virtual bool next(std::string answer) const;
};
}

#endif /* _TASK_BUILDER_S6_RECEIVER_VERIFICATION_STATE_H_ */
