/**
 *  This source file implmenets the Agent abstract class.
 *
 *  Version: 1.4.0
 *  Created on: 04/08/2015
 *  Modified on: 03/01/2017
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/agents/agent.h"

namespace mas
{
namespace agents
{

/**
 * @brief Agent::Agent creates an Agent object without any ID.
 */
Agent::Agent() : location_(new places::Location()) {}

/**
 * @brief Agent::Agent creates an Agent object given its ID, and
 * position (x, y) and orientation (theta).
 * @param id the Agent's identifier number.
 * @param x the Agent's position at x axis.
 * @param y the Agent's position at y axis.
 * @param theta the Agent's orientation at z axis.
 */
Agent::Agent(int id, double x, double y, double theta)
    : id_(id), location_(new places::Location(x, y, theta))
{
}

/**
 * @brief Agent::Agent creates an Agent object given its ID and pose.
 * @param id the Agent's identifier number.
 * @param pose_msg the Agent's pose.
 */
Agent::Agent(int id, const geometry_msgs::Pose& pose_msg)
    : id_(id), location_(new places::Location(pose_msg))
{
}

/**
 * @brief Agent::Agent creates an Agent object given its ID and location.
 * @param id the Agent's identifier number.
 * @param location the Agent's location.
 */
Agent::Agent(int id, const places::Location& location)
    : id_(id), location_(new places::Location(location))
{
}

/**
 * @brief Agent::Agent
 * @param agent
 */
Agent::Agent(const Agent& agent)
    : id_(agent.id_), location_(new places::Location(*agent.location_))
{
}

/**
 * @brief Agent::Agent creates an Agent object given the content of
 * a mas_msgs/Agent ROS message (const pointer).
 * @param agent_msg a mas_msgs/Agent ROS message (const pointer).
 */
Agent::Agent(const mas_msgs::Agent::ConstPtr& agent_msg)
    : id_(agent_msg->id), location_(new places::Location(agent_msg->location))
{
}

/**
 * @brief Agent::Agent creates an Agent object given the content of
 * a mas_msgs/Agent ROS message.
 * @param agent_msg a mas_msgs/Agent ROS message.
 */
Agent::Agent(const mas_msgs::Agent& agent_msg)
    : id_(agent_msg.id), location_(new places::Location(agent_msg.location))
{
}

/**
 * @brief Agent::~Agent destroys this Agent object.
 */
Agent::~Agent()
{
  if (location_)
  {
    delete location_;
    location_ = NULL;
  }
}

/**
 * @brief Agent::getId gives the Agent's ID.
 * @return the Agent's ID.
 */
int Agent::getId() const { return id_; }

/**
 * @brief Agent::getLocation gives the Agent's location
 * @return the Agent's location.
 */
places::Location* Agent::getLocation() const { return location_; }

/**
 * @brief Agent::isValidType verifies the Agent's type.
 * @param type desired Agent type.
 * @return true if this Agent's type matches to the given type.
 */
bool Agent::isValidType(int type) const { return type == getType(); }

/**
 * @brief Agent::getType gives the default Agent type.
 * @return the default Agent type.
 */
int Agent::getType() const { return AGENT_; }

/**
 * @brief Agent::getClassName gives a description of this
 * object class name.
 * @return this object class name.
 */
std::string Agent::getClassName() const { return "agent"; }

/**
 * @brief Agent::setId changes this object ID.
 * @param id the Agent's new ID.
 */
void Agent::setId(int id) { id_ = id; }

/**
 * @brief Agent::setLocation changes the Agent's location.
 * @param x the new position at x axis of this Agent.
 * @param y the new position at y axis of this Agent.
 * @param theta the new orientation at z axis of this Agent.
 */
void Agent::setLocation(double x, double y, double theta)
{
  if (location_)
  {
    location_->setPose(x, y, theta);
    return;
  }
  location_ = new places::Location(x, y, theta);
}

/**
 * @brief Agent::setLocation changes the Agent's location.
 * @param pose_msg the Agent's new pose.
 */
void Agent::setLocation(const geometry_msgs::Pose& pose_msg)
{

  if (location_)
  {
    location_->setPose(pose_msg);
    return;
  }
  location_ = new places::Location(pose_msg);
}

/**
 * @brief Agent::setLocation the Agent's location.
 * @param location the Agent's new location
 */
void Agent::setLocation(const places::Location& location)
{

  if (location_)
  {
    location_->setPose(location);
    return;
  }
  location_ = new places::Location(location);
}

/**
 * @brief Agent::to_msg converts this Agent to a mas_msgs/Agent ROS messsage.
 * @return
 */
mas_msgs::Agent Agent::to_msg() const
{
  mas_msgs::Agent agent_msg;
  agent_msg.id = id_;
  if (location_)
  {
    agent_msg.location = location_->to_msg();
  }
  return agent_msg;
}

/**
 * @brief Agent::str
 * @return
 */
std::string Agent::str() const
{
  std::stringstream aux;
  aux << getClassName() << ": {id: " << id_;
  if (location_)
  {
    aux << ", location: (" << location_->getX() << "," << location_->getY()
        << "," << location_->getTheta() << ")";
  }
  return aux.str();
}

/**
 * @brief Agent::c_str creates a description of this object.
 * @return this object description
 */
const char* Agent::c_str() const { return str().c_str(); }
/**
 * @brief Agent::operator = assigns the given Agent object fields to this
 * object.
 * @param agent an Agent object.
 */
void Agent::operator=(const Agent& agent)
{
  id_ = agent.id_;
  location_->setPose(*agent.location_);
}

/**
 * @brief Agent::operator== compares Agent objects.
 * @param agent an Agent object.
 * @return true if this object is equals to the given Agent object.
 */
bool Agent::operator==(const Agent& agent) const { return id_ == agent.id_; }

/**
 * @brief Agent::operator==
 * @param msg
 * @return
 */
bool Agent::operator==(const mas_msgs::Agent& msg) const
{
  return id_ == msg.id;
}

/**
 * @brief Agent::operator==
 * @param msg
 * @return
 */
bool Agent::operator==(const mas_msgs::Agent::ConstPtr& msg) const
{
  return id_ == msg->id;
}

/**
 * @brief Agent::operator != compares Agent objects.
 * @param agent an Agent object.
 * @return true if this object is not equals to the given Agent object.
 */
bool Agent::operator!=(const Agent& agent) const { return !operator==(agent); }

/**
 * @brief Agent::operator !=
 * @param msg
 * @return
 */
bool Agent::operator!=(const mas_msgs::Agent& msg) const
{
  return !operator==(msg);
}

/**
 * @brief Agent::operator !=
 * @param msg
 * @return
 */
bool Agent::operator!=(const mas_msgs::Agent::ConstPtr& msg) const
{
  return !operator==(msg);
}
}
}
