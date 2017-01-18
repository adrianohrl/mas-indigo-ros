/**
 *  This header file defines the PersonVerificationState class.
 *
 *  Version: 1.4.0
 *  Created on: 11/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_PERSON_VERIFICATION_STATE_H_
#define _TASK_BUILDER_PERSON_VERIFICATION_STATE_H_

#include <mas_srvs/GetPerson.h>
#include <mas/agents/person.h>
#include "task_builder/abstract_state.h"

namespace task_builder
{
class PersonVerificationState : public AbstractState
{

public:
  virtual ~PersonVerificationState();
  virtual bool process(std::string answer);
  virtual std::string str() const = 0;

protected:
  PersonVerificationState(MachineController* controller,
                          std::string question = "From whom?");
  mas::agents::Person* getPerson() const;

private:
  ros::ServiceClient get_person_cli_;
  mas::agents::Person* person_;
  virtual bool next(std::string answer) const = 0;
};
}

#endif /* _TASK_BUILDER_PERSON_VERIFICATION_STATE_H_ */
