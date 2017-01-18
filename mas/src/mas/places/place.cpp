/**
 *  This source file implements the Place class.
 *
 *  Version: 1.4.0
 *  Created on: 03/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/place.h"

namespace mas
{
namespace places
{

/**
 * @brief Place::Place
 * @param id
 * @param name
 * @param boundary
 * @param x
 * @param y
 * @param theta
 */
Place::Place(int id, std::string name, const geometry_msgs::Polygon& boundary,
             double x, double y, double theta)
    : Location(x, y, theta), id_(id), name_(name), boundary_(boundary)
{
}

/**
 * @brief Place::Place
 * @param id
 * @param name
 * @param boundary
 * @param pose_msg
 */
Place::Place(int id, std::string name, const geometry_msgs::Polygon& boundary,
             const geometry_msgs::Pose& pose_msg)
    : Location(pose_msg), id_(id), name_(name), boundary_(boundary)
{
}

/**
 * @brief Place::Place
 * @param place_msg
 */
Place::Place(const mas_msgs::Place::ConstPtr& place_msg)
    : Location(place_msg->location), id_(place_msg->id), name_(place_msg->name),
      boundary_(place_msg->boundary)
{
  if (!isValidType(place_msg->type))
  {
    throw utilities::Exception("Invalid place type.");
  }
}

/**
 * @brief Place::Place
 * @param place_msg
 */
Place::Place(const mas_msgs::Place& place_msg)
    : Location(place_msg.location), id_(place_msg.id), name_(place_msg.name),
      boundary_(place_msg.boundary)
{
  if (!isValidType(place_msg.type))
  {
    throw utilities::Exception("Invalid place type.");
  }
}

/**
 * @brief Place::~Place
 */
Place::~Place() {}

/**
 * @brief Place::getId
 * @return
 */
int Place::getId() const { return id_; }

/**
 * @brief Place::getName
 * @return
 */
std::string Place::getName() const { return name_; }

/**
 * @brief Place::getBoundary
 * @return
 */
geometry_msgs::Polygon Place::getBoundary() const { return boundary_; }

/**
 * @brief Place::isValidType
 * @param type
 * @return
 */
bool Place::isValidType(int type) const { return type == getType(); }

/**
 * @brief Place::getType
 * @return
 */
int Place::getType() const { return PLACE; }

/**
 * @brief Place::setBoundary
 * @param boundary
 */
void Place::setBoundary(geometry_msgs::Polygon boundary)
{
  boundary_ = boundary;
}

/**
 * @brief Place::to_msg
 * @return
 */
mas_msgs::Place Place::to_msg() const
{
  mas_msgs::Place place_msg;
  place_msg.type = getType();
  place_msg.id = id_;
  place_msg.name = name_;
  place_msg.location = Location::to_msg();
  place_msg.boundary = boundary_;
  return place_msg;
}

/**
 * @brief Place::operator =
 * @param place
 */
void Place::operator=(const Place& place)
{
  Location::operator=(place);
  id_ = place.id_;
  name_ = place.name_;
  boundary_ = place.boundary_;
}

/**
 * @brief Place::operator ==
 * @param place
 * @return
 */
bool Place::operator==(const Place& place) const { return id_ == place.id_; }

/**
 * @brief Place::operator !=
 * @param place
 * @return
 */
bool Place::operator!=(const Place& place) const { return !operator ==(place); }
}
}
