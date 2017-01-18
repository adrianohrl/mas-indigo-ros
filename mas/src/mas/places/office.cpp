/**
 *  This source file implements the Office class.
 *
 *  Version: 1.4.0
 *  Created on: 04/04/2016
 *  Modified on: 13/12/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "mas/places/office.h"

namespace mas
{
namespace places
{

/**
 * @brief Office::Office
 * @param id
 * @param name
 * @param floor
 * @param boundary
 * @param x
 * @param y
 * @param theta
 */
Office::Office(int id, std::string name, Floor* floor,
               const geometry_msgs::Polygon& boundary, double x, double y,
               double theta)
    : Place(id, name, boundary, x, y, theta), floor_(floor)
{
}

/**
 * @brief Office::Office
 * @param id
 * @param name
 * @param floor
 * @param boundary
 * @param pose_msg
 */
Office::Office(int id, std::string name, Floor* floor,
               const geometry_msgs::Polygon& boundary,
               const geometry_msgs::Pose& pose_msg)
    : Place(id, name, boundary, pose_msg), floor_(floor)
{
}

/**
 * @brief Office::Office
 * @param office_msg
 */
Office::Office(const mas_msgs::Place::ConstPtr& office_msg)
    : Place(office_msg), floor_(NULL)
{
  // verificar se tem o floor
}

/**
 * @brief Office::Office
 * @param office_msg
 */
Office::Office(const mas_msgs::Place& office_msg)
    : Place(office_msg), floor_(NULL)
{
  // verificar se tem o floor
}

/**
 * @brief Office::~Office
 */
Office::~Office()
{
  if (floor_)
  {
    delete floor_;
    floor_ = NULL;
  }
}

/**
 * @brief Office::getFloor
 * @return
 */
Floor* Office::getFloor() const { return floor_; }

/**
 * @brief Office::getType
 * @return
 */
int Office::getType() const { return OFFICE; }

/**
 * @brief Office::to_msg
 * @return
 */
mas_msgs::Place Office::to_msg() const
{
  mas_msgs::Place floor_msg = Place::to_msg();
  // floor_msg.floor = floor_.to_msg();
  return floor_msg;
}

/**
 * @brief Office::operator =
 * @param office
 */
void Office::operator=(const Office& office)
{
  Place::operator=(office);
  floor_ = office.floor_;
}
}
}
