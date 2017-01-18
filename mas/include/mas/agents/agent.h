/**
 *  This header file defines the Agent abstract class.
 *
 *  Version: 1.4.0
 *  Created on: 05/04/2016
 *  Modified on: 03/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _AGENTS_AGENT_H_
#define _AGENTS_AGENT_H_

#include <sstream>
#include <string>
#include <mas_msgs/Agent.h>
#include "mas/places/location.h"

#define NON_AGENT -1
#define AGENT_ 0
#define PERSON 1
#define COMPUTER 2
#define ROBOT 3
#define USER 4

namespace mas
{
namespace agents
{
class Agent
{

public:
  Agent(int id, double x = 0.0, double y = 0.0, double theta = 0.0);
  Agent(int id, const geometry_msgs::Pose& pose_msg);
  Agent(int id, const mas::places::Location& location);
  Agent(const Agent& agent);
  Agent(const mas_msgs::Agent::ConstPtr& agent_msg);
  Agent(const mas_msgs::Agent& agent_msg);
  virtual ~Agent();

  int getId() const;
  mas::places::Location* getLocation() const;
  void setLocation(double x = 0.0, double y = 0.0, double theta = 0.0);
  void setLocation(const geometry_msgs::Pose& pose_msg);
  void setLocation(const mas::places::Location& location);
  virtual mas_msgs::Agent to_msg() const;
  virtual std::string str() const;
  const char* c_str() const;
  virtual void operator=(const Agent& agent);
  virtual bool operator==(const Agent& agent) const;
  virtual bool operator==(const mas_msgs::Agent& msg) const;
  virtual bool operator==(const mas_msgs::Agent::ConstPtr& msg) const;
  bool operator!=(const Agent& agent) const;
  bool operator!=(const mas_msgs::Agent& msg) const;
  bool operator!=(const mas_msgs::Agent::ConstPtr& msg) const;

protected:
  Agent();

  virtual int getType() const;
  void setId(int id);

private:
  int id_;
  mas::places::Location* location_;

  virtual bool isValidType(int type) const;
  virtual std::string getClassName() const;
};
}
}

#endif /* _AGENTS_AGENT_H_ */
