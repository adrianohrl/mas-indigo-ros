/**
 *  This header file defines the SenderVerificationState pure abstract class.
 *
 *  Version: 1.4.0
 *  Created on: 11/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_SENDER_VERIFICATION_STATE_H_
#define _TASK_BUILDER_SENDER_VERIFICATION_STATE_H_

#include "task_builder/person_verification_state.h"

namespace task_builder
{
class SenderVerificationState : public PersonVerificationState
{
public:
  virtual ~SenderVerificationState();
  virtual bool process(std::string answer);
  virtual std::string str() const = 0;

protected:
  SenderVerificationState(MachineController* controller);

private:
  virtual bool next(std::string answer) const = 0;
};
}

#endif /* _TASK_BUILDER_SENDER_VERIFICATION_STATE_H_ */
