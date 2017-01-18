/**
 *  This source file implements the Computer class.
 *
 *  Version: 1.4.0
 *  Created on: 04/08/2015
 *  Modified on: 03/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/computer.h"

namespace mas
{
namespace agents
{

/**
 * @brief Computer::Computer
 */
Computer::Computer() : Agent() {}

/**
 * @brief Computer::Computer
 * @param id
 * @param hostname
 * @param mobile
 * @param x
 * @param y
 * @param theta
 */
Computer::Computer(int id, std::string hostname, bool mobile, double x,
                   double y, double theta)
    : Agent(id, x, y, theta), hostname_(hostname), mobile_(mobile)
{
}

/**
 * @brief Computer::Computer
 * @param id
 * @param hostname
 * @param mobile
 * @param pose_msg
 */
Computer::Computer(int id, std::string hostname, bool mobile,
                   geometry_msgs::Pose pose_msg)
    : Agent(id, pose_msg), hostname_(hostname), mobile_(mobile)
{
}

/**
 * @brief Computer::Computer
 * @param id
 * @param hostname
 * @param mobile
 * @param location
 */
Computer::Computer(int id, std::string hostname, bool mobile,
                   places::Location location)
    : Agent(id, location), hostname_(hostname), mobile_(mobile)
{
}

/**
 * @brief Computer::Computer
 * @param computer
 */
Computer::Computer(const Computer& computer)
    : Agent(computer), hostname_(computer.hostname_), mobile_(computer.mobile_)
{
}

/**
 * @brief Computer::Computer
 * @param computer_msg
 */
Computer::Computer(const mas_msgs::Agent::ConstPtr& computer_msg)
    : Agent(computer_msg), hostname_(computer_msg->name),
      mobile_(computer_msg->mobile)
{
}

/**
 * @brief Computer::Computer
 * @param computer_msg
 */
Computer::Computer(mas_msgs::Agent computer_msg)
    : Agent(computer_msg), hostname_(computer_msg.name),
      mobile_(computer_msg.mobile)
{
}

/**
 * @brief Computer::~Computer
 */
Computer::~Computer() {}

/**
 * @brief Computer::getHostname
 * @return
 */
std::string Computer::getHostname() const { return hostname_; }

/**
 * @brief Computer::isMobile
 * @return
 */
bool Computer::isMobile() const { return mobile_; }

/**
 * @brief Computer::getType
 * @return
 */
int Computer::getType() const { return COMPUTER; }

/**
 * @brief Computer::getClassName
 * @return
 */
std::string Computer::getClassName() const { return "computer"; }

/**
 * @brief Computer::setHostname
 * @param hostname
 */
void Computer::setHostname(std::string hostname) { hostname_ = hostname; }

/**
 * @brief Computer::setMobile
 * @param mobile
 */
void Computer::setMobile(bool mobile) { mobile_ = mobile; }

/**
 * @brief Computer::to_msg
 * @return
 */
mas_msgs::Agent Computer::to_msg() const
{
  mas_msgs::Agent computer_msg(Agent::to_msg());
  computer_msg.name = hostname_;
  computer_msg.mobile = mobile_;
  return computer_msg;
}

/**
 * @brief Computer::str
 * @return
 */
std::string Computer::str() const
{
  return Agent::str() + ", hostname: " + hostname_ + "}";
}

/**
 * @brief Computer::operator =
 * @param computer
 */
void Computer::operator=(const Computer& computer)
{
  Agent::operator=(computer);
  hostname_ = computer.hostname_;
  mobile_ = computer.mobile_;
}

/**
 * @brief Computer::operator ==
 * @param computer
 * @return
 */
bool Computer::operator==(const Computer& computer) const
{
  return hostname_ == computer.hostname_;
}

/**
 * @brief Computer::operator ==
 * @param msg
 * @return
 */
bool Computer::operator==(const mas_msgs::Agent& msg) const
{
  return hostname_ == msg.computer_hostname;
}

/**
 * @brief Computer::operator ==
 * @param msg
 * @return
 */
bool Computer::operator==(const mas_msgs::Agent::ConstPtr& msg) const
{
  return hostname_ == msg->computer_hostname;
}
}
}
