/**
 *  This source file implements the User class.
 *
 *  Version: 1.4.0
 *  Created on: 04/08/2015
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/user.h"

namespace mas
{
namespace agents
{

/**
 * @brief User::User
 */
User::User()
    : Person(), computer_(NULL), last_beacon_timestamp_(ros::Time::now())
{
}

/**
 * @brief User::User
 * @param id
 * @param name
 * @param hierarchy_level
 * @param login_name
 * @param computer
 * @param x
 * @param y
 * @param theta
 */
User::User(int id, std::string name, HierarchyLevelEnum hierarchy_level,
           std::string login_name, Computer* computer, double x, double y,
           double theta)
    : Person(id, name, hierarchy_level, x, y, theta), computer_(computer),
      last_beacon_timestamp_(ros::Time::now())
{
  login_name_ = login_name;
}

/**
 * @brief User::User
 * @param id
 * @param name
 * @param hierarchy_level
 * @param login_name
 * @param computer
 * @param pose_msg
 */
User::User(int id, std::string name, HierarchyLevelEnum hierarchy_level,
           std::string login_name, Computer* computer,
           const geometry_msgs::Pose& pose_msg)
    : Person(id, name, hierarchy_level, pose_msg), login_name_(login_name),
      computer_(computer), last_beacon_timestamp_(ros::Time::now())
{
}

/**
 * @brief User::User
 * @param id
 * @param name
 * @param hierarchy_level
 * @param login_name
 * @param computer
 * @param location
 */
User::User(int id, std::string name, HierarchyLevelEnum hierarchy_level,
           std::string login_name, Computer* computer,
           const places::Location& location)
    : Person(id, name, hierarchy_level, location), login_name_(login_name),
      computer_(computer), last_beacon_timestamp_(ros::Time::now())
{
}

/**
 * @brief User::User
 * @param user
 */
User::User(const User& user)
    : Person(user), login_name_(user.login_name_), computer_(user.computer_),
      last_beacon_timestamp_(user.last_beacon_timestamp_)
{
}

/**
 * @brief User::User
 * @param user_msg
 */
User::User(const mas_msgs::Agent::ConstPtr& user_msg)
    : Person(user_msg), login_name_(user_msg->login_name), computer_(NULL),
      last_beacon_timestamp_(ros::Time::now())
{
  mas_msgs::Agent computer_msg;
  computer_msg.type = COMPUTER;
  computer_msg.id = user_msg->computer_id;
  computer_msg.name = user_msg->computer_hostname;
  computer_msg.mobile = user_msg->computer_mobile;
  computer_msg.location = user_msg->computer_location;
  computer_ = new Computer(computer_msg);
}

/**
 * @brief User::User
 * @param user_msg
 */
User::User(const mas_msgs::Agent& user_msg)
    : Person(user_msg), login_name_(user_msg.login_name), computer_(NULL),
      last_beacon_timestamp_(ros::Time::now())
{
  mas_msgs::Agent computer_msg;
  computer_msg.type = COMPUTER;
  computer_msg.id = user_msg.computer_id;
  computer_msg.name = user_msg.computer_hostname;
  computer_msg.mobile = user_msg.computer_mobile;
  computer_msg.location = user_msg.computer_location;
  computer_ = new Computer(computer_msg);
}

/**
 * @brief User::~User
 */
User::~User() {}

/**
 * @brief User::getLoginName
 * @return
 */
std::string User::getLoginName() const { return login_name_; }

/**
 * @brief User::getComputer
 * @return
 */
Computer* User::getComputer() const { return computer_; }

/**
 * @brief User::getLastBeaconTimestamp
 * @return
 */
ros::Time User::getLastBeaconTimestamp() const
{
  return last_beacon_timestamp_;
}

/**
 * @brief User::isValid
 * @param password
 * @return
 */
bool User::isValid(std::string password) const { return password_ == password; }

/**
 * @brief User::isLogged
 * @return
 */
bool User::isLogged() const
{
  return (ros::Time::now() - last_beacon_timestamp_).toSec() <=
         MAXIMUM_USER_BEACON_ABSENCE_DURATION;
}

/**
 * @brief User::isNotLoggedAnyMore
 * @param user
 * @return
 */
bool User::isNotLoggedAnyMore(User* user) { return !user->isLogged(); }

/**
 * @brief User::getType
 * @return
 */
int User::getType() const { return USER; }

/**
 * @brief User::getClassName
 * @return
 */
std::string User::getClassName() const { return "user"; }

/**
 * @brief User::setLoginName
 * @param login_name
 */
void User::setLoginName(std::string login_name) { login_name_ = login_name; }

/**
 * @brief User::setPassword
 * @param password
 */
void User::setPassword(std::string password) { password_ = password; }

/**
 * @brief User::setComputer
 * @param computer
 */
void User::setComputer(Computer* computer)
{
  computer_ = computer;
  Agent::setLocation(*computer->getLocation());
}

/**
 * @brief User::setLastBeaconTimestamp
 * @param last_beacon_timestamp
 */
void User::setLastBeaconTimestamp(ros::Time last_beacon_timestamp)
{
  if (last_beacon_timestamp > last_beacon_timestamp_ &&
      last_beacon_timestamp <= ros::Time::now())
  {
    last_beacon_timestamp_ = last_beacon_timestamp;
  }
}

/**
 * @brief User::to_msg
 * @return
 */
mas_msgs::Agent User::to_msg() const
{
  mas_msgs::Agent user_msg(Person::to_msg());
  user_msg.login_name = login_name_;
  if (computer_)
  {
    mas_msgs::Agent computer_msg(computer_->to_msg());
    user_msg.computer_id = computer_msg.id;
    user_msg.computer_hostname = computer_msg.name;
    user_msg.computer_mobile = computer_msg.mobile;
    user_msg.computer_location = computer_msg.location;
  }
  return user_msg;
}

/**
 * @brief User::str
 * @return
 */
std::string User::str() const
{
  std::string aux = Person::str();
  return aux.substr(0, aux.length() - 1) + ", login: " + login_name_ +
         (computer_ ? ", " + computer_->str() : "") + "}";
}

/**
 * @brief User::operator =
 * @param user
 */
void User::operator=(const User& user)
{
  Person::operator=(user);
  login_name_ = user.login_name_;
  computer_ = user.computer_;
}
}
}
