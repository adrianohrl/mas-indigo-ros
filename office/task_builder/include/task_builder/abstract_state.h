/**
 *  This header file defines the AbstractState pure abstract class.
 *
 *  Version: 1.4.0
 *  Created on: 10/05/2016
 *  Modified on: 13/12/2016
 *  Authors: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *           Luís Victor Pessiqueli Bonin (luis-bonin@hotmail.com)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_BUILDER_ABSTRACT_STATE_H_
#define _TASK_BUILDER_ABSTRACT_STATE_H_

#include <string>
#include <ros/ros.h>

namespace task_builder
{
class MachineController;

class AbstractState
{
public:
  virtual ~AbstractState();
  std::string getQuestion() const;
  std::string getMessage() const;
  bool isFinalState() const;
  void setQuestion(std::string question);
  void setMessage(std::string message);
  virtual bool process(std::string answer) = 0;
  virtual std::string str() const = 0;
  const char* c_str() const;

protected:
  AbstractState(MachineController* controller, std::string question = "",
                bool final_state = false);
  ros::NodeHandle* getNodeHandle() const;
  MachineController* getController() const;

private:
  MachineController* controller_;
  std::string question_;
  std::string message_;
  bool final_state_;
  virtual bool next(std::string answer) const = 0;
};
}

#endif /* _TASK_BUILDER_ABSTRACT_STATE_H_ */
