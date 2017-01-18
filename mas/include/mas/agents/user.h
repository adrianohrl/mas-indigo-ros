/**
 *  This header file defines the User class.
 *
 *  Version: 1.4.0
 *  Created on: 05/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _AGENTS_USER_H_
#define _AGENTS_USER_H_

#include <ros/ros.h>
#include <mas_msgs/User.h>
#include "mas/agents/person.h"
#include "mas/agents/computer.h"

#define USER_BEACON_INTERVAL_DURATION 5.0
#define MAXIMUM_USER_BEACON_ABSENCE_DURATION 3 * USER_BEACON_INTERVAL_DURATION

namespace mas
{
namespace agents
{
class User : public Person
{

public:
  User();
  User(int id, std::string name, HierarchyLevelEnum hierarchy_level,
       std::string login_name, Computer* computer, double x = 0.0,
       double y = 0.0, double theta = 0.0);
  User(int id, std::string name, HierarchyLevelEnum hierarchy_level,
       std::string login_name, Computer* computer,
       const geometry_msgs::Pose& pose_msg);
  User(int id, std::string name, HierarchyLevelEnum hierarchy_level,
       std::string login_name, Computer* computer,
       const mas::places::Location& location);
  User(const User& user);
  User(const mas_msgs::Agent::ConstPtr& user_msg);
  User(const mas_msgs::Agent& user_msg);
  virtual ~User();

  std::string getLoginName() const;
  Computer* getComputer() const;
  ros::Time getLastBeaconTimestamp() const;
  bool isValid(std::string password) const;
  bool isLogged() const;
  static bool isNotLoggedAnyMore(User* user);
  void setComputer(Computer* computer);
  void
  setLastBeaconTimestamp(ros::Time last_beacon_timestamp = ros::Time::now());
  virtual mas_msgs::Agent to_msg() const;
  virtual std::string str() const;
  virtual void operator=(const User& user);

protected:
  virtual int getType() const;
  void setLoginName(std::string login_name);
  void setPassword(std::string password);

private:
  std::string login_name_;
  std::string password_;
  Computer* computer_;
  ros::Time last_beacon_timestamp_;

  virtual std::string getClassName() const;
};

struct UserComparator
{
  User* user_;
  UserComparator(User* user) : user_(user) {}
  bool operator()(User* user) { return *user_ == *user; }
  bool operator()(const User& user) { return *user_ == user; }
};
}
}

#endif /* _AGENTS_USER_H_ */
