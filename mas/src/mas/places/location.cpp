/**
 *  This source file implements the Location source.
 *
 *  Version: 1.4.0
 *  Created on: 03/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/location.h"

namespace mas
{
namespace places
{

/**
 * @brief Location::Location
 * @param x
 * @param y
 * @param theta
 */
Location::Location(double x, double y, double theta)
    : x_(x), y_(y), theta_(theta)
{
}

/**
 * @brief Location::Location
 * @param location
 */
Location::Location(const Location &location)
  : x_(location.x_), y_(location.y_), theta_(location.theta_)
{
}

/**
 * @brief Location::Location
 * @param pose_msg
 */
Location::Location(const geometry_msgs::Pose& pose_msg)
    : x_(pose_msg.position.x), y_(pose_msg.position.y),
      theta_(tf::getYaw(pose_msg.orientation))
{
}

/**
 * @brief Location::Location
 * @param location_msg
 */
Location::Location(const mas_msgs::Location::ConstPtr& location_msg)
    : x_(location_msg->x), y_(location_msg->y), theta_(location_msg->theta)
{
}

/**
 * @brief Location::Location
 * @param location_msg
 */
Location::Location(const mas_msgs::Location& location_msg)
    : x_(location_msg.x), y_(location_msg.y), theta_(location_msg.theta)
{
}

/**
 * @brief Location::~Location
 */
Location::~Location() {}

/**
 * @brief Location::getX
 * @return
 */
double Location::getX() const { return x_; }

/**
 * @brief Location::getY
 * @return
 */
double Location::getY() const { return y_; }

/**
 * @brief Location::getTheta
 * @return
 */
double Location::getTheta() const { return theta_; }

/**
 * @brief Location::getPose
 * @return
 */
geometry_msgs::Pose Location::getPose() const
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = x_;
  pose_msg.position.y = y_;
  pose_msg.orientation = tf::createQuaternionMsgFromYaw(theta_);
  return pose_msg;
}

/**
 * @brief Location::setPose
 * @param x
 * @param y
 * @param theta
 */
void Location::setPose(double x, double y, double theta)
{
  x_ = x;
  y_ = y;
  theta_ = theta;
}

/**
 * @brief Location::setPose
 * @param pose_msg
 */
void Location::setPose(const geometry_msgs::Pose& pose_msg)
{
  x_ = pose_msg.position.x;
  y_ = pose_msg.position.y;
  theta_ = tf::getYaw(pose_msg.orientation);
}

/**
 * @brief Location::setPose
 * @param location
 */
void Location::setPose(const mas_msgs::Location& location_msg)
{
  x_ = location_msg.x;
  y_ = location_msg.y;
  theta_ = location_msg.theta;
}

/**
 * @brief Location::setPose
 * @param location_msg
 */
void Location::setPose(const Location& location)
{
  x_ = location.x_;
  y_ = location.y_;
  theta_ = location.theta_;
}

/**
 * @brief Location::to_msg
 * @return
 */
mas_msgs::Location Location::to_msg() const
{
  mas_msgs::Location location_msg;
  location_msg.x = x_;
  location_msg.y = y_;
  location_msg.theta = theta_;
  return location_msg;
}

/**
 * @brief Location::operator =
 * @param location
 */
void Location::operator=(const Location& location)
{
  x_ = location.x_;
  y_ = location.y_;
  theta_ = location.theta_;
}
}
}
