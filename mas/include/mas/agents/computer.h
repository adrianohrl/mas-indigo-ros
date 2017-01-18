/**
 *  This header file defines the Computer class.
 *
 *  Version: 1.4.0
 *  Created on: 05/04/2016
 *  Modified on: 03/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _AGENTS_COMPUTER_H_
#define _AGENTS_COMPUTER_H_

#include <mas_msgs/Computer.h>
#include "mas/agents/agent.h"

namespace mas
{
namespace agents
{
class Computer : public Agent
{

public:
  Computer();
  Computer(int id, std::string hostname, bool mobile = false, double x = 0.0,
           double y = 0.0, double theta = 0.0);
  Computer(int id, std::string hostname, bool mobile,
           geometry_msgs::Pose pose_msg);
  Computer(int id, std::string hostname, bool mobile,
           mas::places::Location location);
  Computer(const Computer& computer);
  Computer(const mas_msgs::Agent::ConstPtr& computer_msg);
  Computer(mas_msgs::Agent computer_msg);
  virtual ~Computer();

  std::string getHostname() const;
  bool isMobile() const;
  virtual mas_msgs::Agent to_msg() const;
  virtual std::string str() const;
  virtual void operator=(const Computer& computer);
  virtual bool operator==(const Computer& computer) const;
  virtual bool operator==(const mas_msgs::Agent& msg) const;
  virtual bool operator==(const mas_msgs::Agent::ConstPtr& msg) const;

protected:
  virtual int getType() const;
  void setHostname(std::string hostname);
  void setMobile(bool mobile);

private:
  std::string hostname_;
  bool mobile_;

  virtual std::string getClassName() const;
};
}
}

#endif /* _AGENTS_COMPUTER_H_ */
